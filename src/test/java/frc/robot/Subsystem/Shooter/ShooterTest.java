package frc.robot.Subsystem.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Subsystem.SubsystemTestBase;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ShooterTest extends SubsystemTestBase {
    private static final double kEpsilon = 1e-9;
    private static final double kSpinUpToleranceRps = 1.0;

    @BeforeAll
    static void createSubsystems() {
        warmUp(Shooter::get);
    }

    // ---- setpoints ----

    @Test
    void shotIsVelocityAtShotSpeed() {
        assertEquals(Mode.VELOCITY, Shooter.SHOT.mode);
        assertEquals(ShooterConstants.ShotSpeed.baseUnitMagnitude(), Shooter.SHOT.baseUnits, kEpsilon);
        assertEquals(30.0, ShooterConstants.ShotSpeed.in(RotationsPerSecond), kEpsilon);
    }

    @Test
    void ferryIsVelocityAtFerrySpeed() {
        assertEquals(Mode.VELOCITY, Shooter.FERRY.mode);
        assertEquals(ShooterConstants.FerrySpeed.baseUnitMagnitude(), Shooter.FERRY.baseUnits, kEpsilon);
    }

    @Test
    void ferryIsSlowerThanShot() {
        assertTrue(ShooterConstants.FerrySpeed.in(RotationsPerSecond) < ShooterConstants.ShotSpeed.in(RotationsPerSecond));
    }

    @Test
    void idleCoasts() {
        assertSame(Setpoint.COAST, Shooter.IDLE);
        assertEquals(Mode.IDLE, Shooter.IDLE.mode);
    }

    // ---- behavior in sim ----

    @Test
    void getReturnsSameInstance() {
        assertSame(Shooter.get(), Shooter.get());
    }

    @Test
    void setpointCommandAppliesSetpoint() {
        Shooter shooter = Shooter.get();
        schedule(shooter.setpointCommand(Shooter.FERRY));
        assertSame(Shooter.FERRY, shooter.getSetpoint());

        schedule(shooter.setpointCommand(Shooter.IDLE));
        assertSame(Shooter.IDLE, shooter.getSetpoint());
    }

    @Test
    void spinsUpToShotSpeed() {
        Shooter shooter = Shooter.get();
        schedule(shooter.setpointCommand(Shooter.SHOT));

        runUntil(
            () -> shooter.nearVelocity(ShooterConstants.ShotSpeed, RotationsPerSecond.of(kSpinUpToleranceRps)),
            3.0,
            "Shooter never reached " + ShooterConstants.ShotSpeed.in(RotationsPerSecond) + " RPS, last read "
                + shooter.getVelocity().in(RotationsPerSecond));
        assertTrue(shooter.spunUp(RotationsPerSecond.of(kSpinUpToleranceRps)));
    }

    @Test
    void spinsDownWhenIdle() {
        Shooter shooter = Shooter.get();
        schedule(shooter.setpointCommand(Shooter.SHOT));
        runFor(1.0);

        schedule(shooter.setpointCommand(Shooter.IDLE));
        runUntil(
            () -> shooter.getVelocity().in(RotationsPerSecond) < ShooterConstants.ShotSpeed.in(RotationsPerSecond) - 5.0,
            3.0,
            "Shooter didn't slow down after going idle, still at " + shooter.getVelocity().in(RotationsPerSecond));
        // not spun up while coasting, even if the wheel is still turning
        assertTrue(!shooter.spunUp());
    }
}
