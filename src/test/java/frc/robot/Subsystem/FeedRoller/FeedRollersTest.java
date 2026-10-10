package frc.robot.Subsystem.FeedRoller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.io.MotorIO.Mode;
import frc.robot.Ports;
import frc.robot.Subsystem.SubsystemTestBase;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.FeedRollers.FeedRollersConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class FeedRollersTest extends SubsystemTestBase {
    private static final double kEpsilon = 1e-9;
    private static final double kToleranceRps = 1.5;

    @BeforeAll
    static void createSubsystems() {
        warmUp(FeedRollers::get);
    }

    // ---- setpoints ----

    @Test
    void idleIsZeroVelocity() {
        assertEquals(Mode.VELOCITY, FeedRollers.IDLE.mode);
        assertEquals(0.0, FeedRollers.IDLE.baseUnits, kEpsilon);
    }

    @Test
    void runningIsVelocityAtRunningSpeed() {
        assertEquals(Mode.VELOCITY, FeedRollers.RUNNING.mode);
        assertEquals(FeedRollersConstants.RunningSpeed.baseUnitMagnitude(), FeedRollers.RUNNING.baseUnits, kEpsilon);
    }

    @Test
    void runningSpeedIsSeventyPercentOfShotSpeedReversed() {
        assertEquals(-70.0, FeedRollersConstants.RunningSpeed.in(RotationsPerSecond), kEpsilon);
        assertEquals(0.7, FeedRollersConstants.PercentOfLUT, kEpsilon);
    }

    // ---- behavior in sim ----

    @Test
    void getReturnsSameInstance() {
        assertSame(FeedRollers.get(), FeedRollers.get());
    }

    @Test
    void setpointCommandAppliesSetpoint() {
        FeedRollers rollers = FeedRollers.get();
        schedule(rollers.setpointCommand(FeedRollers.RUNNING));
        assertSame(FeedRollers.RUNNING, rollers.getSetpoint());

        schedule(rollers.setpointCommand(FeedRollers.IDLE));
        assertSame(FeedRollers.IDLE, rollers.getSetpoint());
    }

    @Test
    void reachesRunningSpeed() {
        FeedRollers rollers = FeedRollers.get();
        schedule(rollers.setpointCommand(FeedRollers.RUNNING));

        runUntil(
            () -> rollers.nearVelocity(FeedRollersConstants.RunningSpeed, RotationsPerSecond.of(kToleranceRps)),
            3.0,
            "Feed rollers never reached " + FeedRollersConstants.RunningSpeed.in(RotationsPerSecond)
                + " RPS, last read " + rollers.getVelocity().in(RotationsPerSecond));
        assertTrue(rollers.getVelocity().in(RotationsPerSecond) < 0.0, "running should spin backwards");
    }

    // ---- follower (ID 19 follows the main roller, ID 14, spinning the opposite way) ----

    // Read-only handles to the real devices, same as Robot's dashboard motors
    private static final TalonFX kMain = new TalonFX(Ports.FEEDER_ROLLERS.id, Ports.FEEDER_ROLLERS.bus);
    private static final TalonFX kFollower = new TalonFX(Ports.FEED_ROLLERS_FOLLOWER.id, Ports.FEED_ROLLERS_FOLLOWER.bus);

    private static double volts(TalonFX motor) {
        return motor.getMotorVoltage().refresh().getValueAsDouble();
    }

    @Test
    void followerIsItsOwnMotor() {
        assertNotEquals(Ports.FEEDER_ROLLERS.id, Ports.FEED_ROLLERS_FOLLOWER.id, "follower can't share the main roller's CAN ID");
        for (Ports port : Ports.values()) {
            if (port != Ports.FEED_ROLLERS_FOLLOWER && port.bus.getName().equals(Ports.FEED_ROLLERS_FOLLOWER.bus.getName())) {
                assertNotEquals(port.id, Ports.FEED_ROLLERS_FOLLOWER.id, "CAN ID 19 is also used by " + port);
            }
        }
    }

    @Test
    void followerIsInFollowerMode() {
        runFor(0.2);
        var mode = kFollower.getControlMode().refresh().getValue();
        assertTrue(mode.toString().contains("Follower"), "follower motor should be in a follower control mode, got " + mode);
    }

    @Test
    void followerDrivesOppositeWhileRunning() {
        FeedRollers rollers = FeedRollers.get();
        schedule(rollers.setpointCommand(FeedRollers.RUNNING));
        runUntil(() -> Math.abs(volts(kMain)) > 1.0, 3.0, "main roller never got voltage");
        runFor(0.5);

        double main = volts(kMain);
        double follower = volts(kFollower);
        System.out.printf("Feed rollers running: main %.2f V %.1f rps %.1f A | follower %.2f V %.1f rps %.1f A | mode %s%n",
            main, kMain.getVelocity().refresh().getValueAsDouble(), kMain.getStatorCurrent().refresh().getValueAsDouble(),
            follower, kFollower.getVelocity().refresh().getValueAsDouble(), kFollower.getStatorCurrent().refresh().getValueAsDouble(),
            kFollower.getControlMode().refresh().getValue());
        assertTrue(Math.abs(follower) > 1.0, "follower isn't getting any voltage (main has " + main + " V)");
        assertTrue(Math.signum(follower) == -Math.signum(main), "follower should drive the opposite way of the main roller");
        // Not checking the follower's voltage size: the lib's sim spins the follower the same way as the main roller
        // (it ignores Opposed), so in sim the follower fights the roller and its voltage/current aren't realistic.

        schedule(rollers.setpointCommand(FeedRollers.IDLE));
        runFor(1.0);
    }

    @Test
    void followerStopsWithMain() {
        FeedRollers rollers = FeedRollers.get();
        schedule(rollers.setpointCommand(FeedRollers.RUNNING));
        runFor(1.0);
        schedule(rollers.setpointCommand(FeedRollers.IDLE));
        runUntil(() -> Math.abs(rollers.getVelocity().in(RotationsPerSecond)) < kToleranceRps, 3.0, "main roller didn't stop");
        runFor(0.5);
        assertEquals(0.0, volts(kFollower), 0.5, "follower still has voltage after stopping");
    }

    @Test
    void followerGetsSameConfig() {
        var config = FeedRollersConstants.getConfig();
        assertEquals(config.mainConfig.Slot1.kP, config.followerConfig.Slot1.kP, kEpsilon);
        assertEquals(config.mainConfig.CurrentLimits.StatorCurrentLimit, config.followerConfig.CurrentLimits.StatorCurrentLimit, kEpsilon);
        assertEquals(config.mainConfig.CurrentLimits.SupplyCurrentLimit, config.followerConfig.CurrentLimits.SupplyCurrentLimit, kEpsilon);
    }

    @Test
    void stopsWhenIdle() {
        FeedRollers rollers = FeedRollers.get();
        schedule(rollers.setpointCommand(FeedRollers.RUNNING));
        runFor(1.0);

        schedule(rollers.setpointCommand(FeedRollers.IDLE));
        runUntil(
            () -> Math.abs(rollers.getVelocity().in(RotationsPerSecond)) < kToleranceRps,
            3.0,
            "Feed rollers didn't stop, still at " + rollers.getVelocity().in(RotationsPerSecond));
    }
}
