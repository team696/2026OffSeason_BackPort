"""
Shooter simulator / LUT generator for the 2026 REBUILT robot.

  numpy  - vector math
  scipy  - trajectory ODE (drag + Magnus backspin lift), root finding for the RPS that hits the target
  sympy  - derives how much the flywheel slows down when a ball is pulled through (energy transfer)
  simpy  - discrete-event sim of a rapid-fire burst (feeder timing vs. flywheel recovery w/ current limits)

Run:   python tools/shooter_sim/shooter_sim.py              (LUTs + airtime maps + burst sim)
       python tools/shooter_sim/shooter_sim.py --plot       (also saves trajectory plots, needs matplotlib)
       python tools/shooter_sim/shooter_sim.py --burst-distance 4.5 --burst-balls 10
       python tools/shooter_sim/shooter_sim.py --gearing 0.48889 --set RPS_OFFSET=1.5 --set HUB_HOOD_OFFSET_DEG=-2

All the knobs you are meant to change are in the TUNING KNOBS block below; any UPPERCASE constant
can also be overridden with --set NAME=VALUE.

Values marked  [CODE]  come from the robot code; [WEB] from the FUEL spec; [GUESS] are physical
assumptions you should calibrate against 2-3 real shots once you have the robot.
"""

import argparse
import math
from dataclasses import dataclass

import numpy as np
import simpy
import sympy as sp
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

# ============================================================================================
# TUNING KNOBS  -  change these (or override from the command line:  --set NAME=VALUE)
# ============================================================================================
# --- Output corrections: nudge the LUT after real shots without touching the physics ---------
#   LUT rps  = sim rps  * RPS_SCALE  + RPS_OFFSET      + <mode>_RPS_OFFSET
#   LUT hood = sim hood * HOOD_SCALE + HOOD_OFFSET_DEG + <mode>_HOOD_OFFSET_DEG
RPS_SCALE = 1.00  # 1.03 = everything 3% faster
RPS_OFFSET = 0.0  # rps added to every shot
HOOD_SCALE = 1.00
HOOD_OFFSET_DEG = 0.0  # deg added to every hood angle
HUB_RPS_OFFSET = 0.0
HUB_HOOD_OFFSET_DEG = 0.0
FERRY_RPS_OFFSET = 0.0
FERRY_HOOD_OFFSET_DEG = 0.0
AIRTIME_SCALE = 1.00  # airtime map = sim time * scale + offset (tunes motion-comp lead)
AIRTIME_OFFSET_S = 0.0

# --- Distance / aim offsets -------------------------------------------------------------------
DISTANCE_OFFSET_M = 0.0  # true distance = measured (LUT key) + this. + if vision/odometry reads short
HUB_AIM_X_OFFSET = 0.0  # m, + aims past hub centre, - aims short of it
HUB_AIM_Z_OFFSET = 0.0  # m, raise/lower the plane the ball must fall through
FERRY_AIM_X_OFFSET = 0.0  # m, + lands ferry shots farther

# --- Real shots used for calibration -------------------------------------------------------------
# (distance robot centre -> hub centre m, flywheel rps as Phoenix reported it, hood deg, ...)
# Leave empty to use the values below as written. Add more shots as you test - they get averaged.
# CALIBRATE picks what the real shots solve for:
#   "launch_at_home" - keep SLIP_EFFICIENCY, solve the exit angle with the hood at home (use while
#                      you don't know the hood's real exit angles)
#   "slip"           - keep LAUNCH_AT_HOME_DEG, solve SLIP_EFFICIENCY (use once you've measured it)
CALIBRATE = "launch_at_home"
# A shot can be hit by a flat or a steep launch. "steep" = hood near home is a lob (flattens as
# it rises), which is the only choice that leaves the hood's 30 deg of travel usable for the hub.
CALIBRATION_ROOT = "steep"
CALIBRATION_SHOTS = [
    (3.0, 32.5, 48.0 / 18.0),  # [USER] ~3 m scored at 30-35 rps, hood at its lowest shooting angle (old "48" / 18)
]

# --- Physics calibration factors (fit these to 2-3 real shots) ---------------------------------
SLIP_EFFICIENCY = 0.90  # [GUESS] fraction of ideal no-slip exit speed. Shots short everywhere -> raise
ENERGY_EFFICIENCY = 0.80  # [GUESS] fraction of flywheel energy lost that ends up in the ball
DRAG_CD = 0.50  # [GUESS] smooth sphere, subcritical Re (~5e4-1e5)
MAGNUS_SCALE = 1.0  # [GUESS] multiplier on backspin lift; 0 = no lift
SPIN_DECAY_TAU = 4.0  # [GUESS] s, backspin e-folding time
# Exit angle (above horizontal) = LAUNCH_AT_HOME_DEG + HOOD_LAUNCH_SIGN * (hood - HOOD_HOME_DEG)
#                                 + HOOD_LAUNCH_OFFSET_DEG
LAUNCH_AT_HOME_DEG = 77.0  # [GUESS] exit angle with hood at home; overwritten by "launch_at_home" calibration
HOOD_LAUNCH_SIGN = -1.0  # [USER] hood moving up makes the shot FLATTER
HOOD_LAUNCH_OFFSET_DEG = 0.0  # extra fine-tune on the exit angle

# --- Selection preferences -------------------------------------------------------------------
SIGMA_RPS = 0.75  # expected rps error on the robot
SIGMA_HOOD_DEG = 0.75  # expected hood error
MIN_ENTRY_ANGLE_DEG = 35.0  # ball must be falling at least this steeply into the hub
FERRY_MIN_APEX = 2.5  # m, lob height for ferry shots
FERRY_LANDING_TOL = 0.5  # m, how close a ferry shot must land
USABLE_FRACTION = 0.90  # use <= 90% of free speed so the wheel can recover after a ball
# How to pick between the many (hood, rps) pairs that hit the target:
#   "low_rps"    - lowest rps whose expected miss is still small enough (MIN_MARGIN_SIGMAS).
#                  Keeps headroom for recovery, shorter flight time. Matches what worked IRL.
#   "max_margin" - most forgiving shot, even if it needs a lot of speed (steep lobs)
SHOT_PREFERENCE = "low_rps"
MIN_MARGIN_SIGMAS = 2.0  # usable opening must be >= this many 1-sigma misses (2 -> ~95% in; 3 makes the LUT jumpy)

# ============================================================================================
# ROBOT CONFIG
# ============================================================================================
IN = 0.0254
LB = 0.45359237
G = 9.806  # [CODE] FieldSimConstants.kGravity

# --- Ball (FUEL) -------------------------------------------------------------------------------
BALL_DIAMETER_IN = 5.91  # [WEB] AndyMark am-5801 / game manual
BALL_MASS = 0.215  # [WEB] 0.448-0.5 lb (203-227 g)
AIR_DENSITY = 1.20

# --- Shooter (flywheel) --------------------------------------------------------------------------
SHOOTER_GEARING = 45.0 / 22.0  # [CODE] kGearing, motor turns per drum turn (<1 = overdrive) CHECK!
SHOOTER_MOTORS = 2  # [CODE] main + opposed follower, Kraken X60
WHEEL_DIAMETER_IN = 3.0  # [USER] fixed drum
FLYWHEEL_MASS_LB = 3.5  # [USER] each side
FLYWHEEL_COUNT = 2
FLYWHEEL_DIAMETER_IN = 3.0  # [USER]
FLYWHEEL_SHAPE_K = 0.5  # I = k*m*r^2 ; 0.5 solid disk, ~0.8-0.9 for rim-weighted wheels
OTHER_INERTIA = 0.0012143  # [CODE] ShooterConstants sim MOI (shaft, drum, pulleys)

# How the ball is driven:
#   "hood" - drum + hood: ball centre moves at ~HALF drum surface speed, gets backspin
#   "dual" - ball pinched between two wheels at the same speed: full surface speed, ~no spin
CONTACT = "hood"
# 2 in hood roller surface speed / drum surface speed. 0 = free/passive roller,
# >0 = driven forward (ball speed = (v_drum + v_roller)/2, less backspin)
HOOD_ROLLER_SURFACE_RATIO = 1.0  # [GUESS] IRL 30-35 rps @ 3 m means the ball leaves near full drum
#                                   speed, i.e. the roller is driven. Calibration absorbs the rest.

# Kraken X60 with FOC (MotionMagicVelocityVoltage .withEnableFOC(true))  [CODE] MotorIOTalonFX
MOTOR_STALL_TORQUE = 9.37
MOTOR_STALL_CURRENT = 483.0
MOTOR_FREE_CURRENT = 2.0
MOTOR_FREE_RPM = 5800.0
BATTERY_V = 12.0
STATOR_LIMIT = 90.0  # [CODE]
SUPPLY_LIMIT = 45.0  # [CODE]
KV_VOLTS_PER_RPS = 0.245  # [CODE] Slot1 kV (mechanism rps, tuned at 45/22)
KP_VOLTS_PER_RPS = 0.5  # [CODE] Slot1 kP
MM_ACCEL = 300.0  # [CODE] MotionMagicAcceleration, rps/s

# --- Hood --------------------------------------------------------------------------------------
# Drive train [USER]: 12T motor pulley -> 24T pulley, 10 DP pinion on that shaft -> 180T hood sector
HOOD_MOTOR_PULLEY_T = 12
HOOD_SHAFT_PULLEY_T = 24
HOOD_PINION_T = 10  # [CHECK] tooth count of the 10 DP pinion (10T fits your 540 deg reading)
HOOD_SECTOR_T = 180
HOOD_CODE_GEARING = 2.0  # [CODE] HoodConstants.kGearing (what Phoenix currently uses)
HOOD_CODE_READING_FULL_TRAVEL = 540.0  # [USER] deg Phoenix shows across the full hood travel
HOOD_HOME_DEG = 0.0  # [USER] hood reads 0 at the homing hard stop (HoodConstants.kMinAngle)
HOOD_MIN_ABOVE_HOME_DEG = 48.0 / 18.0  # [USER] old "48 deg" min in 2:1 units = 2.67 real deg above home
#                                       (HoodConstants.kMinShootingAngle)
HOOD_MAX_DEG = 33.0  # [USER] measured max (HoodConstants.kMaxAngle); None = home + travel from the reading above
HOOD_SEARCH_STEP = 1.0

# --- Robot / field -----------------------------------------------------------------------------
SHOOTER_X_OFFSET = 0.252  # [CODE] FieldSimConstants.kShooterOffset (forward of robot centre)
SHOOTER_Z = 0.668  # [CODE] exit height
HUB_Z = 72.0 * IN  # [CODE] FieldLayout.kHubAltitude
HUB_RADIUS = 0.56  # [CODE] FieldLayout.kHubRadius

# LUT keys - same distances that are already in LookUpTables.java (robot centre -> target, m)
HUB_DISTANCES = [1.5, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5]
NEUTRAL_FERRY_DISTANCES = [3.0, 4.5, 6.0, 7.5]
OPPOSITE_FERRY_DISTANCES = [7.0, 9.0, 11.0, 13.0]


def recompute():
    """Derived values - rerun after changing any knob."""
    global BALL_RADIUS, BALL_INERTIA, BALL_AREA, WHEEL_RADIUS, SHOOTER_INERTIA, MOTOR_FREE_SPEED
    global R_MOTOR, KT, KV_MOTOR, HOOD_GEARING, HOOD_TRAVEL_DEG, HOOD_SEARCH_MIN, HOOD_SEARCH_MAX
    global SPEED_RATIO_EXPR, SPEED_RATIO
    BALL_RADIUS = BALL_DIAMETER_IN * IN / 2
    BALL_INERTIA = 0.4 * BALL_MASS * BALL_RADIUS**2  # solid high-density foam sphere
    BALL_AREA = math.pi * BALL_RADIUS**2
    WHEEL_RADIUS = WHEEL_DIAMETER_IN * IN / 2
    fly_r = FLYWHEEL_DIAMETER_IN * IN / 2
    SHOOTER_INERTIA = FLYWHEEL_COUNT * FLYWHEEL_SHAPE_K * FLYWHEEL_MASS_LB * LB * fly_r**2 + OTHER_INERTIA
    MOTOR_FREE_SPEED = MOTOR_FREE_RPM / 60.0 * 2 * math.pi
    R_MOTOR = BATTERY_V / MOTOR_STALL_CURRENT
    KT = MOTOR_STALL_TORQUE / MOTOR_STALL_CURRENT
    KV_MOTOR = MOTOR_FREE_SPEED / (BATTERY_V - R_MOTOR * MOTOR_FREE_CURRENT)
    HOOD_GEARING = (HOOD_SHAFT_PULLEY_T / HOOD_MOTOR_PULLEY_T) * (HOOD_SECTOR_T / HOOD_PINION_T)
    HOOD_TRAVEL_DEG = HOOD_CODE_READING_FULL_TRAVEL * HOOD_CODE_GEARING / HOOD_GEARING
    HOOD_SEARCH_MIN = HOOD_HOME_DEG + HOOD_MIN_ABOVE_HOME_DEG
    HOOD_SEARCH_MAX = HOOD_MAX_DEG if HOOD_MAX_DEG is not None else HOOD_HOME_DEG + HOOD_TRAVEL_DEG
    SPEED_RATIO_EXPR, SPEED_RATIO = derive_ball_exchange()


def max_usable_rps():
    return USABLE_FRACTION * MOTOR_FREE_SPEED / (2 * math.pi) / SHOOTER_GEARING


def lut_rps(sim_rps, mode):
    return sim_rps * RPS_SCALE + RPS_OFFSET + (HUB_RPS_OFFSET if mode == "hub" else FERRY_RPS_OFFSET)


def lut_hood(sim_hood, mode):
    return sim_hood * HOOD_SCALE + HOOD_OFFSET_DEG + (HUB_HOOD_OFFSET_DEG if mode == "hub" else FERRY_HOOD_OFFSET_DEG)


def lut_airtime(sim_t):
    return sim_t * AIRTIME_SCALE + AIRTIME_OFFSET_S


# ============================================================================================
# SymPy: flywheel speed after a ball goes through
# ============================================================================================
def contact_factors():
    """(ball speed / wheel surface speed, ball spin * R_ball / ball speed) for the CONTACT type."""
    if CONTACT == "dual":
        return 1.0, 0.0
    r = HOOD_ROLLER_SURFACE_RATIO
    return (1.0 + r) / 2.0, (1.0 - r) / (1.0 + r)


def derive_ball_exchange():
    I, w0, w1, m, Ib, Rb, rw, es, ee, c, k = sp.symbols("I w0 w1 m I_b R_b r_w eta_s eta_e c k", positive=True)
    v = es * c * w1 * rw  # ball centre speed (fraction c of wheel surface speed, with slip)
    wb = k * v / Rb  # ball backspin (rolling on the hood -> k = 1)
    energy = sp.Eq(ee * sp.Rational(1, 2) * I * (w0**2 - w1**2), sp.Rational(1, 2) * m * v**2 + sp.Rational(1, 2) * Ib * wb**2)
    w1_sol = [s for s in sp.solve(energy, w1) if s.is_positive is not False][0]
    w1_sol = sp.simplify(w1_sol)
    ratio = sp.simplify(w1_sol / w0)
    f = sp.lambdify((I, m, Ib, Rb, rw, es, ee, c, k), ratio, "math")
    return ratio, f(SHOOTER_INERTIA, BALL_MASS, BALL_INERTIA, BALL_RADIUS, WHEEL_RADIUS, SLIP_EFFICIENCY,
                    ENERGY_EFFICIENCY, *contact_factors())


def exit_speed(rps_before):
    """Ball exit speed (m/s) for a flywheel at rps_before when the ball enters."""
    w_after = rps_before * 2 * math.pi * SPEED_RATIO
    return SLIP_EFFICIENCY * w_after * WHEEL_RADIUS * contact_factors()[0]


def hood_to_launch_deg(hood_deg):
    return LAUNCH_AT_HOME_DEG + HOOD_LAUNCH_SIGN * (hood_deg - HOOD_HOME_DEG) + HOOD_LAUNCH_OFFSET_DEG


# ============================================================================================
# SciPy: trajectory
# ============================================================================================
def _rhs(t, s):
    x, z, vx, vz, wb = s
    v = math.hypot(vx, vz)
    if v < 1e-6:
        return [vx, vz, 0.0, -G, 0.0]
    q = 0.5 * AIR_DENSITY * BALL_AREA * v / BALL_MASS
    spin = BALL_RADIUS * wb / v
    cl = MAGNUS_SCALE * (1.0 / (2.0 + 1.0 / spin) if spin > 1e-3 else 0.0)  # Sawicki et al. lift model
    ax = -q * DRAG_CD * vx - q * cl * vz  # backspin lift is perpendicular to v, rotated "up"
    az = -q * DRAG_CD * vz + q * cl * vx - G
    return [vx, vz, ax, az, -wb / SPIN_DECAY_TAU]


@dataclass
class Shot:
    x_cross: float  # horizontal position (from shooter exit) where ball falls through target_z
    t_cross: float
    apex: float
    entry_deg: float  # angle below horizontal at the crossing
    sol: object


def fly(v, launch_deg, target_z, t_max=8.0):
    th = math.radians(launch_deg)
    s0 = [0.0, SHOOTER_Z, v * math.cos(th), v * math.sin(th), contact_factors()[1] * v / BALL_RADIUS]

    def cross(t, s):
        return s[1] - target_z

    cross.terminal, cross.direction = True, -1
    sol = solve_ivp(_rhs, (0, t_max), s0, events=cross, dense_output=True, rtol=1e-7, atol=1e-9)
    if len(sol.t_events[0]) == 0:
        return None
    te = sol.t_events[0][0]
    xe, ze, vxe, vze, _ = sol.y_events[0][0]
    apex = float(np.max(sol.y[1]))
    if apex <= target_z + 1e-6 and SHOOTER_Z <= target_z:
        return None
    return Shot(xe, te, apex, math.degrees(math.atan2(-vze, vxe)), sol)


def clears_front_rim(shot, x_target):
    """Ball bottom must be above the hub top when passing over the near edge of the opening."""
    x_front = x_target - HUB_RADIUS
    ts = np.linspace(0, shot.t_cross, 400)
    xs, zs = shot.sol.sol(ts)[0], shot.sol.sol(ts)[1]
    if x_front <= 0:
        return True
    return np.interp(x_front, xs, zs) - BALL_RADIUS > HUB_Z


# ============================================================================================
# Calibration: fit SLIP_EFFICIENCY or LAUNCH_AT_HOME_DEG so the sim reproduces real shots that scored
# ============================================================================================
def _calibrate_launch_at_home(dist, rps, hood):
    global LAUNCH_AT_HOME_DEG
    x_target, target_z, _ = target_for(dist, "hub")

    def err(home):
        global LAUNCH_AT_HOME_DEG
        LAUNCH_AT_HOME_DEG = home
        sh = fly(exit_speed(rps), hood_to_launch_deg(hood), target_z)
        return -x_target if sh is None else sh.x_cross - x_target

    grid = np.arange(20.0, 89.5, 1.0)
    vals = [err(h) for h in grid]
    roots = [brentq(err, a, b, xtol=1e-3) for a, b, fa, fb in zip(grid[:-1], grid[1:], vals[:-1], vals[1:])
             if fa * fb < 0]
    if not roots:
        raise SystemExit(f"Calibration: no exit angle makes {dist} m @ {rps} rps hit - check SLIP_EFFICIENCY")
    print(f"  {dist} m @ {rps} rps hood {hood:.2f}: exit angle at home could be "
          + " or ".join(f"{r:.1f}" for r in roots) + f" deg -> using {CALIBRATION_ROOT}")
    return max(roots) if CALIBRATION_ROOT == "steep" else min(roots)


def _calibrate_slip(dist, rps, hood):
    x_target, target_z, _ = target_for(dist, "hub")

    def err(slip):
        global SLIP_EFFICIENCY
        SLIP_EFFICIENCY = slip
        recompute()
        sh = fly(exit_speed(rps), hood_to_launch_deg(hood), target_z)
        return -x_target if sh is None else sh.x_cross - x_target

    return brentq(err, 0.05, 3.0, xtol=1e-4)


def calibrate():
    global SLIP_EFFICIENCY, LAUNCH_AT_HOME_DEG
    if not CALIBRATION_SHOTS:
        return
    shots = ", ".join(f"{d} m @ {r} rps hood {h:.2f}" for d, r, h in CALIBRATION_SHOTS)
    if CALIBRATE == "launch_at_home":
        LAUNCH_AT_HOME_DEG = float(np.mean([_calibrate_launch_at_home(*s) for s in CALIBRATION_SHOTS]))
        print(f"Calibrated LAUNCH_AT_HOME_DEG = {LAUNCH_AT_HOME_DEG:.1f} from {shots} "
              f"(SLIP_EFFICIENCY held at {SLIP_EFFICIENCY}) -> hood {HOOD_SEARCH_MIN:.1f}-{HOOD_SEARCH_MAX:.1f} deg "
              f"= exit {hood_to_launch_deg(HOOD_SEARCH_MIN):.1f}-{hood_to_launch_deg(HOOD_SEARCH_MAX):.1f} deg")
    else:
        SLIP_EFFICIENCY = float(np.mean([_calibrate_slip(*s) for s in CALIBRATION_SHOTS]))
        recompute()
        print(f"Calibrated SLIP_EFFICIENCY = {SLIP_EFFICIENCY:.3f} from {shots}")
    print(f"  -> ball leaves at {SLIP_EFFICIENCY * contact_factors()[0] * SPEED_RATIO * 100:.0f}% of drum surface speed")


# ============================================================================================
# LUT solver
# ============================================================================================
def solve_rps(hood_deg, x_target, target_z):
    """RPS that makes the ball fall through target_z exactly at x_target (None if unreachable)."""
    launch = hood_to_launch_deg(hood_deg)

    def err(rps):
        s = fly(exit_speed(rps), launch, target_z)
        return -x_target if s is None else s.x_cross - x_target

    grid = np.linspace(2.0, max_usable_rps(), 30)
    vals = [err(r) for r in grid]
    for a, b, fa, fb in zip(grid[:-1], grid[1:], vals[:-1], vals[1:]):
        if fa < 0 <= fb:
            return brentq(err, a, b, xtol=1e-4)
    return None


@dataclass
class Solution:
    distance: float
    hood: float
    rps: float
    exit_v: float
    tof: float
    apex: float
    entry_deg: float
    miss_sigma: float  # expected miss distance (m) from SIGMA_RPS / SIGMA_HOOD_DEG
    score_margin: float  # opening half-width / miss_sigma  (bigger = more forgiving)
    lut_rps: float = 0.0  # after output corrections - what goes in the Java LUT
    lut_hood: float = 0.0
    lut_tof: float = 0.0


def evaluate(distance, hood, rps, x_target, target_z, opening):
    launch = hood_to_launch_deg(hood)
    shot = fly(exit_speed(rps), launch, target_z)
    d_r, d_h = 0.25, 0.25
    xp = fly(exit_speed(rps + d_r), launch, target_z)
    xm = fly(exit_speed(rps - d_r), launch, target_z)
    hp = fly(exit_speed(rps), hood_to_launch_deg(hood + d_h), target_z)
    hm = fly(exit_speed(rps), hood_to_launch_deg(hood - d_h), target_z)
    if None in (shot, xp, xm, hp, hm):
        return None
    dxdr = (xp.x_cross - xm.x_cross) / (2 * d_r)
    dxdh = (hp.x_cross - hm.x_cross) / (2 * d_h)
    miss = math.hypot(dxdr * SIGMA_RPS, dxdh * SIGMA_HOOD_DEG)
    return shot, Solution(distance, hood, rps, exit_speed(rps), shot.t_cross, shot.apex, shot.entry_deg, miss, opening / max(miss, 1e-6))


def target_for(distance, mode):
    """(horizontal x from shooter exit, z plane, allowed miss) for a measured robot distance."""
    true_d = distance + DISTANCE_OFFSET_M - SHOOTER_X_OFFSET
    if mode == "hub":
        return true_d + HUB_AIM_X_OFFSET, HUB_Z + HUB_AIM_Z_OFFSET, HUB_RADIUS - BALL_RADIUS
    return true_d + FERRY_AIM_X_OFFSET, BALL_RADIUS, FERRY_LANDING_TOL


def build_lut(distances, mode):
    out, prev_hood = [], None
    for d in distances:
        x_target, target_z, opening = target_for(d, mode)
        cands = []
        for hood in np.arange(HOOD_SEARCH_MIN, HOOD_SEARCH_MAX + 1e-9, HOOD_SEARCH_STEP):
            rps = solve_rps(hood, x_target, target_z)
            if rps is None:
                continue
            r = evaluate(d, float(hood), rps, x_target, target_z, opening)
            if r is None:
                continue
            shot, sol = r
            if mode == "hub" and (sol.entry_deg < MIN_ENTRY_ANGLE_DEG or not clears_front_rim(shot, x_target)):
                continue
            if mode == "ferry" and sol.apex < FERRY_MIN_APEX:
                continue
            cands.append(sol)
        if not cands:
            out.append(None)
            print(f"  {d:5.2f} m : NO VALID SHOT (out of range for hood {HOOD_SEARCH_MIN:.1f}-{HOOD_SEARCH_MAX:.1f} deg, <= {max_usable_rps():.1f} rps)")
            continue
        accurate = [c for c in cands if c.score_margin >= MIN_MARGIN_SIGMAS]
        if SHOT_PREFERENCE == "low_rps" and accurate:
            lowest = min(c.rps for c in accurate)
            good = [c for c in accurate if c.rps <= lowest + 0.5]  # near-lowest: keep hood smooth
        else:
            best = max(c.score_margin for c in cands)
            good = [c for c in cands if c.score_margin >= 0.95 * best]
        pick = min(good, key=lambda c: abs(c.hood - prev_hood)) if prev_hood is not None else max(good, key=lambda c: c.score_margin)
        prev_hood = pick.hood
        pick.lut_rps, pick.lut_hood, pick.lut_tof = lut_rps(pick.rps, mode), lut_hood(pick.hood, mode), lut_airtime(pick.tof)
        out.append(pick)
        print(f"  {d:5.2f} m : hood {pick.lut_hood:5.1f} deg (launch {hood_to_launch_deg(pick.hood):5.1f}, "
              f"hood motor {pick.lut_hood / 360 * HOOD_GEARING:5.2f} rot)  {pick.lut_rps:6.2f} rps (motor {pick.lut_rps*SHOOTER_GEARING:6.2f})  "
              f"exit {pick.exit_v:5.2f} m/s  tof {pick.tof:4.2f} s  apex {pick.apex:4.2f} m  "
              f"entry {pick.entry_deg:4.1f} deg  miss(1sig) {pick.miss_sigma*100:4.1f} cm")
    return out


# ============================================================================================
# SimPy: rapid-fire burst (flywheel recovery between balls)
# ============================================================================================
class Flywheel:
    DT = 0.001

    def __init__(self, env):
        self.env, self.w, self.target_rps, self.mm_rps = env, 0.0, 0.0, 0.0
        self.log = []

    def run(self):
        while True:
            # MotionMagicVelocityVoltage: profiled setpoint at MM_ACCEL, then kV*ref + kP*error
            step = MM_ACCEL * self.DT
            self.mm_rps += max(-step, min(step, self.target_rps - self.mm_rps))
            rps = self.w / (2 * math.pi)
            # code's kV/kP are per mechanism rps at 45/22; rescale if simulating a different gearing
            scale = SHOOTER_GEARING / (45.0 / 22.0)
            volts = KV_VOLTS_PER_RPS * scale * self.mm_rps + KP_VOLTS_PER_RPS * scale * (self.mm_rps - rps)
            volts = max(-BATTERY_V, min(BATTERY_V, volts))
            w_motor = self.w * SHOOTER_GEARING
            i = (volts - w_motor / KV_MOTOR) / R_MOTOR
            i = max(-STATOR_LIMIT, min(STATOR_LIMIT, i))
            duty = abs(volts) / BATTERY_V
            if duty > 1e-3 and abs(i) * duty > SUPPLY_LIMIT:
                i = math.copysign(SUPPLY_LIMIT / duty, i)
            torque = SHOOTER_MOTORS * KT * i * SHOOTER_GEARING
            self.w += torque / SHOOTER_INERTIA * self.DT
            self.log.append((self.env.now, rps, i))
            yield self.env.timeout(self.DT)


def burst_sim(sol, n_balls, bps, ready_tol):
    env = simpy.Environment()
    fw = Flywheel(env)
    env.process(fw.run())
    x_target, target_z, _ = target_for(sol.distance, "hub")
    results, ready = [], {}

    def operator():
        fw.target_rps = sol.lut_rps  # what the robot commands (with your output corrections)
        t0 = env.now
        while abs(fw.w / (2 * math.pi) - sol.lut_rps) > ready_tol:  # wait for Shooter at setpoint
            yield env.timeout(Flywheel.DT)
        ready["t"] = env.now - t0
        for k in range(n_balls):
            rps_in = fw.w / (2 * math.pi)
            shot = fly(exit_speed(rps_in), hood_to_launch_deg(sol.lut_hood), target_z)
            fw.w *= SPEED_RATIO  # ball pulls energy out of the wheel (SymPy result)
            err = None if shot is None else shot.x_cross - x_target
            results.append((k + 1, env.now, rps_in, err))
            yield env.timeout(1.0 / bps)

    env.process(operator())
    env.run(until=0.5 + 6.0 + n_balls / bps)
    return ready.get("t"), results


# ============================================================================================
# Output
# ============================================================================================
def java_block(var, sols, attr, unit):
    rows = [f"            {var}.put({s.distance}, {getattr(s, attr):.2f}); // m, {unit}" for s in sols if s is not None]
    return "\n".join(rows)


def apply_overrides(pairs):
    """--set NAME=VALUE for any UPPERCASE knob above (numbers, None, or text like CONTACT=dual)."""
    g = globals()
    for pair in pairs:
        name, _, raw = pair.partition("=")
        name = name.strip().upper()
        if name not in g:
            raise SystemExit(f"--set: unknown knob {name}")
        raw = raw.strip()
        if raw == "None":
            val = None
        else:
            try:
                val = eval(raw, {"__builtins__": {}})  # allows 22/45, [2.0, 3.0] etc.
                val = float(val) if isinstance(val, (int, float)) else val
            except Exception:
                val = raw
        if isinstance(g[name], int) and isinstance(val, float) and val.is_integer():
            val = int(val)
        g[name] = val


def main():
    global CONTACT, WHEEL_DIAMETER_IN, FLYWHEEL_DIAMETER_IN, SHOOTER_GEARING, HOOD_ROLLER_SURFACE_RATIO, HOOD_MAX_DEG
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true", help="save trajectory plots (needs matplotlib)")
    ap.add_argument("--burst-distance", type=float, default=3.5)
    ap.add_argument("--burst-balls", type=int, default=8)
    ap.add_argument("--bps", type=float, default=6.0, help="balls per second the feeder delivers")
    ap.add_argument("--ready-tol", type=float, default=1.0, help="rps tolerance before feeding")
    ap.add_argument("--contact", choices=["hood", "dual"], default=CONTACT)
    ap.add_argument("--wheel-diameter", type=float, default=WHEEL_DIAMETER_IN, help="contact drum, inches")
    ap.add_argument("--flywheel-diameter", type=float, default=FLYWHEEL_DIAMETER_IN, help="3.5 lb wheels, inches")
    ap.add_argument("--gearing", type=lambda x: float(eval(x, {"__builtins__": {}})), default=SHOOTER_GEARING,
                    help="motor rotations per drum rotation (<1 = overdrive). Code says 45/22 = 2.045")
    ap.add_argument("--hood-roller-ratio", type=float, default=HOOD_ROLLER_SURFACE_RATIO)
    ap.add_argument("--hood-max", type=float, default=HOOD_MAX_DEG)
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override any knob, e.g. --set RPS_OFFSET=1.5 --set DISTANCE_OFFSET_M=-0.1 (repeatable)")
    args = ap.parse_args()
    CONTACT, WHEEL_DIAMETER_IN, FLYWHEEL_DIAMETER_IN = args.contact, args.wheel_diameter, args.flywheel_diameter
    SHOOTER_GEARING, HOOD_ROLLER_SURFACE_RATIO, HOOD_MAX_DEG = args.gearing, args.hood_roller_ratio, args.hood_max
    apply_overrides(args.set)
    recompute()
    calibrate()

    print("=" * 100)
    print(f"Contact: {CONTACT} | wheel {WHEEL_RADIUS*2/IN:.1f} in | Ball: {BALL_DIAMETER_IN:.2f} in, {BALL_MASS*1000:.0f} g | shooter inertia {SHOOTER_INERTIA*1e4:.1f} kg*cm^2 "
          f"| gearing {SHOOTER_GEARING:.3f}:1 | max usable {max_usable_rps():.1f} drum rps")
    print(f"SymPy: w_after / w_before = {SPEED_RATIO_EXPR}  ->  {SPEED_RATIO:.4f} "
          f"({(1-SPEED_RATIO)*100:.1f}% speed drop per ball)")
    print(f"Exit speed at 30 rps: {exit_speed(30):.2f} m/s, at {max_usable_rps():.1f} rps: {exit_speed(max_usable_rps()):.2f} m/s")
    print("RPS below = DRUM rps (real wheel speed). Phoenix reports this only if SensorToMechanismRatio matches the real gearing.")
    print(f"Hood: {HOOD_MOTOR_PULLEY_T}T->{HOOD_SHAFT_PULLEY_T}T x {HOOD_PINION_T}T->{HOOD_SECTOR_T}T = {HOOD_GEARING:.1f}:1 "
          f"| real travel {HOOD_TRAVEL_DEG:.1f} deg ({HOOD_CODE_READING_FULL_TRAVEL:.0f} deg read at kGearing {HOOD_CODE_GEARING}) "
          f"| search {HOOD_SEARCH_MIN:.1f}-{HOOD_SEARCH_MAX:.1f} deg")
    print(f"      -> set HoodConstants.kGearing = {HOOD_GEARING:.1f} so the hood reads real degrees")
    print(f"Corrections: rps x{RPS_SCALE} {RPS_OFFSET:+} (hub {HUB_RPS_OFFSET:+}, ferry {FERRY_RPS_OFFSET:+}) | "
          f"hood x{HOOD_SCALE} {HOOD_OFFSET_DEG:+} (hub {HUB_HOOD_OFFSET_DEG:+}, ferry {FERRY_HOOD_OFFSET_DEG:+}) | "
          f"dist {DISTANCE_OFFSET_M:+} m | aim hub x{HUB_AIM_X_OFFSET:+} z{HUB_AIM_Z_OFFSET:+}, ferry x{FERRY_AIM_X_OFFSET:+} | "
          f"airtime x{AIRTIME_SCALE} {AIRTIME_OFFSET_S:+} s")
    print("=" * 100)

    print("\nHUB")
    hub = build_lut(HUB_DISTANCES, "hub")
    print("\nNEUTRAL ZONE FERRY")
    neutral = build_lut(NEUTRAL_FERRY_DISTANCES, "ferry")
    print("\nOPPOSITE SIDE FERRY")
    opposite = build_lut(OPPOSITE_FERRY_DISTANCES, "ferry")

    java = f"""
// ---------------- LookUpTables.java ----------------
        // ScoringVelocityLUT
{java_block('scoringvelocityLUT', hub, 'lut_rps', 'RPS')}
        // ScoringHoodLUT
{java_block('scoringhoodLUT', hub, 'lut_hood', 'Degrees')}
        // NetrualZoneVelocityLUT
{java_block('neturalzonevelocityLUT', neutral, 'lut_rps', 'RPS')}
        // NeturalZoneHoodLUT
{java_block('neturalzonehoodLUT', neutral, 'lut_hood', 'Degrees')}
        // OppositeSideVelocityLUT
{java_block('oppositesidevelocityLUT', opposite, 'lut_rps', 'RPS')}
        // OppositeSideHoodLUT
{java_block('oppositesidehoodLUT', opposite, 'lut_hood', 'Degrees')}

// ---------------- AirtimeRegression.java ----------------
        // hub
""" + "\n".join(f"        insertPointHub(Meter.of({s.distance}), Second.of({s.lut_tof:.3f}));" for s in hub if s) + """
        // ferry neutral
""" + "\n".join(f"        insertPointFerryNeutral(Meter.of({s.distance}), Second.of({s.lut_tof:.3f}));" for s in neutral if s)
    print(java)
    out_path = __file__.replace("shooter_sim.py", "lut_output.txt")
    with open(out_path, "w") as f:
        f.write(java)
    print(f"\n(saved to {out_path})")

    # ---- SimPy burst ----
    pick = min((s for s in hub if s), key=lambda s: abs(s.distance - args.burst_distance), default=None)
    if pick:
        t_ready, res = burst_sim(pick, args.burst_balls, args.bps, args.ready_tol)
        print(f"\nBURST @ {pick.distance} m, {args.bps} balls/s, commanded {pick.lut_rps:.2f} rps / hood {pick.lut_hood:.1f} deg")
        print(f"  spin-up from 0 to within {args.ready_tol} rps: {t_ready:.2f} s" if t_ready else "  never reached setpoint")
        ok = HUB_RADIUS - BALL_RADIUS
        for k, t, rps_in, err in res:
            verdict = "MISS(short)" if err is None else ("SCORE" if abs(err) <= ok else ("MISS long" if err > 0 else "MISS short"))
            e = "  n/a " if err is None else f"{err*100:+6.1f}"
            print(f"  ball {k:2d}  t={t:5.2f}s  wheel {rps_in:5.2f} rps  miss {e} cm  {verdict}")

    if args.plot:
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(11, 5))
        for s in hub:
            if s:
                x_t, z_t, _ = target_for(s.distance, "hub")
                shot = fly(s.exit_v, hood_to_launch_deg(s.hood), z_t)
                ts = np.linspace(0, shot.t_cross, 200)
                xs, zs = shot.sol.sol(ts)[0], shot.sol.sol(ts)[1]
                ax.plot(xs - (x_t - HUB_AIM_X_OFFSET), zs, label=f"{s.distance} m")
        ax.axhline(HUB_Z, color="k", lw=0.8)
        ax.axvspan(-HUB_RADIUS, HUB_RADIUS, color="gray", alpha=0.2)
        ax.set_xlabel("x relative to hub centre (m)")
        ax.set_ylabel("z (m)")
        ax.set_aspect("equal")
        ax.legend()
        p = __file__.replace("shooter_sim.py", "hub_trajectories.png")
        fig.savefig(p, dpi=120, bbox_inches="tight")
        print(f"\nplot saved to {p}")


if __name__ == "__main__":
    main()
