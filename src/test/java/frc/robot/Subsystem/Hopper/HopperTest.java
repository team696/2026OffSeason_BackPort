package frc.robot.Subsystem.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Subsystem.SubsystemTestBase;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class HopperTest extends SubsystemTestBase {
    private static final double kEpsilon = 1e-9;

    @BeforeAll
    static void createSubsystems() {
        warmUp(Hopper.Rollers::get);
    }

    // ---- setpoints ----

    @Test
    void intakeSetpointsAreVoltage() {
        assertEquals(Mode.VOLTAGE, Hopper.Rollers.INTAKE.mode);
        assertEquals(Mode.VOLTAGE, Hopper.Rollers.SLOW_INTAKE.mode);
        assertEquals(Mode.VOLTAGE, Hopper.Rollers.OUTAKE.mode);
        assertEquals(HopperConstants.Rollers.kIntakeVoltage.in(Volts), Hopper.Rollers.INTAKE.baseUnits, kEpsilon);
        assertEquals(HopperConstants.Rollers.kSlowIntakeVoltage.in(Volts), Hopper.Rollers.SLOW_INTAKE.baseUnits, kEpsilon);
        assertEquals(HopperConstants.Rollers.kOuttakeVoltage.in(Volts), Hopper.Rollers.OUTAKE.baseUnits, kEpsilon);
    }

    @Test
    void voltagesPointTheRightWay() {
        double intake = HopperConstants.Rollers.kIntakeVoltage.in(Volts);
        double slow = HopperConstants.Rollers.kSlowIntakeVoltage.in(Volts);
        double outtake = HopperConstants.Rollers.kOuttakeVoltage.in(Volts);
        assertTrue(intake > 0.0);
        assertTrue(slow > 0.0 && slow < intake, "slow intake should be slower than intake");
        assertTrue(outtake < 0.0, "outtake should run backwards");
        assertTrue(Math.abs(intake) <= 12.0 && Math.abs(outtake) <= 12.0, "voltages must fit the battery");
    }

    @Test
    void idleCoasts() {
        assertSame(Setpoint.COAST, Hopper.Rollers.IDLE);
    }

    // ---- behavior in sim ----

    @Test
    void getReturnsSameInstance() {
        assertSame(Hopper.Rollers.get(), Hopper.Rollers.get());
    }

    @Test
    void setpointCommandAppliesSetpoint() {
        Hopper.Rollers rollers = Hopper.Rollers.get();
        schedule(rollers.setpointCommand(Hopper.Rollers.INTAKE));
        assertSame(Hopper.Rollers.INTAKE, rollers.getSetpoint());

        schedule(rollers.setpointCommand(Hopper.Rollers.IDLE));
        assertSame(Hopper.Rollers.IDLE, rollers.getSetpoint());
    }

    @Test
    void intakeAndOuttakeSpinOppositeWays() {
        Hopper.Rollers rollers = Hopper.Rollers.get();

        schedule(rollers.setpointCommand(Hopper.Rollers.INTAKE));
        runUntil(() -> Math.abs(rollers.getVelocity().in(RotationsPerSecond)) > 1.0, 3.0, "Hopper didn't spin on INTAKE");
        double intakeDirection = Math.signum(rollers.getVelocity().in(RotationsPerSecond));

        schedule(rollers.setpointCommand(Hopper.Rollers.OUTAKE));
        runUntil(
            () -> Math.signum(rollers.getVelocity().in(RotationsPerSecond)) == -intakeDirection
                && Math.abs(rollers.getVelocity().in(RotationsPerSecond)) > 1.0,
            3.0,
            "Hopper didn't reverse on OUTAKE");

        schedule(rollers.setpointCommand(Hopper.Rollers.IDLE));
    }
}
