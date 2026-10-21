package frc.robot.shooting;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import frc.lib.util.tunablenumbers.TunableNumber;
import frc.robot.subsystems.Hood.HoodConstants;

/**
 * Fudge factors for the shooting LUTs, editable live from SmartDashboard (TunableNumbers/Shooting/...).
 * Shooting short -> raise RPS Scale or an RPS Offset (or Distance Offset).
 */
public class ShotTuning {
    // final rps = LUT rps * RPS Scale + RPS Offset (hub or pass)
    public static final TunableNumber rpsScale = new TunableNumber("Shooting/RPS Scale", 0.5);
    public static final TunableNumber hubRpsOffset = new TunableNumber("Shooting/Hub RPS Offset", -1.0); // was overshooting a little
    public static final TunableNumber passRpsOffset = new TunableNumber("Shooting/Pass RPS Offset", 0.0);
    // final hood = LUT hood + Hood Offset (hub or pass), then clamped to the hood's shooting range
    public static final TunableNumber hubHoodOffset = new TunableNumber("Shooting/Hub Hood Offset", 0.0);
    public static final TunableNumber passHoodOffset = new TunableNumber("Shooting/Pass Hood Offset", 0.0);
    // meters added to the measured distance before the LUT lookup (+ if the robot thinks it's closer than it is)
    public static final TunableNumber distanceOffset = new TunableNumber("Shooting/Distance Offset", 0.0);

    /** Call once at startup so every fudge factor shows up on the dashboard before the first shot. */
    public static void init() {}

    public static double applyRps(double lutRps, boolean isHub) {
        return lutRps * rpsScale.getAsDouble() + (isHub ? hubRpsOffset : passRpsOffset).getAsDouble();
    }

    public static double applyHood(double lutHoodDeg, boolean isHub) {
        return MathUtil.clamp(
            lutHoodDeg + (isHub ? hubHoodOffset : passHoodOffset).getAsDouble(),
            HoodConstants.kMinShootingAngle.in(Degrees),
            HoodConstants.kMaxAngle.in(Degrees));
    }
}
