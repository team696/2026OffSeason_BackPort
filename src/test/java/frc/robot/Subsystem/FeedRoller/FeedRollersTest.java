package frc.robot.Subsystem.FeedRoller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.io.MotorIO.Mode;
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
        assertEquals(-21.0, FeedRollersConstants.RunningSpeed.in(RotationsPerSecond), kEpsilon);
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
