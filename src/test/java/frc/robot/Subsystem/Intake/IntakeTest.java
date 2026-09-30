package frc.robot.Subsystem.Intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Subsystem.SubsystemTestBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class IntakeTest extends SubsystemTestBase {
    private static final double kEpsilon = 1e-9;

    @BeforeAll
    static void createSubsystems() {
        warmUp(() -> {
            Intake.RackAndPinion.get();
            Intake.Rollers.get();
        });
    }

    // ---- rack and pinion setpoints ----

    @Test
    void rackSetpointsArePositionControl() {
        assertEquals(Mode.POSITIONPID, Intake.RackAndPinion.RETRACT.mode);
        assertEquals(Mode.POSITIONPID, Intake.RackAndPinion.EXTEND.mode);
        assertEquals(Mode.POSITIONPID, Intake.RackAndPinion.HALF.mode);
    }

    @Test
    void rackSetpointsMatchConstants() {
        assertEquals(IntakeConstants.RackAndPinion.kInAngle.baseUnitMagnitude(), Intake.RackAndPinion.RETRACT.baseUnits, kEpsilon);
        assertEquals(IntakeConstants.RackAndPinion.kOutAngle.baseUnitMagnitude(), Intake.RackAndPinion.EXTEND.baseUnits, kEpsilon);
        assertEquals(Intake.RackAndPinion.EXTEND.baseUnits / 2.0, Intake.RackAndPinion.HALF.baseUnits, kEpsilon);
    }

    @Test
    void rackSetpointsAreOrdered() {
        assertTrue(Intake.RackAndPinion.RETRACT.baseUnits < Intake.RackAndPinion.HALF.baseUnits);
        assertTrue(Intake.RackAndPinion.HALF.baseUnits < Intake.RackAndPinion.EXTEND.baseUnits);
    }

    @Test
    void angleDistanceConversionRoundTrips() {
        var converter = IntakeConstants.RackAndPinion.converter;
        assertEquals(14.0, converter.toDistance(IntakeConstants.RackAndPinion.kOutAngle).in(Inches), 1e-6);
        assertEquals(-0.01, converter.toDistance(IntakeConstants.RackAndPinion.kInAngle).in(Inches), 1e-6);
    }

    // ---- roller setpoints ----

    @Test
    void rollerSetpointsAreVelocity() {
        assertEquals(Mode.VELOCITY, Intake.Rollers.INTAKE.mode);
        assertEquals(Mode.VELOCITY, Intake.Rollers.OUTAKE.mode);
        assertEquals(IntakeConstants.Roller.kIntakeSpeed.baseUnitMagnitude(), Intake.Rollers.INTAKE.baseUnits, kEpsilon);
        assertEquals(IntakeConstants.Roller.kOuttakeSpeed.baseUnitMagnitude(), Intake.Rollers.OUTAKE.baseUnits, kEpsilon);
    }

    @Test
    void outtakeIsIntakeReversed() {
        assertEquals(
            -IntakeConstants.Roller.kIntakeSpeed.in(RotationsPerSecond),
            IntakeConstants.Roller.kOuttakeSpeed.in(RotationsPerSecond),
            kEpsilon);
    }

    @Test
    void rollerIdleCoasts() {
        assertSame(Setpoint.COAST, Intake.Rollers.IDLE);
    }

    // ---- behavior in sim ----

    @Test
    void getReturnsSameInstance() {
        assertSame(Intake.RackAndPinion.get(), Intake.RackAndPinion.get());
        assertSame(Intake.Rollers.get(), Intake.Rollers.get());
    }

    @Test
    void rackExtendsAndRetracts() {
        Intake.RackAndPinion rack = Intake.RackAndPinion.get();

        schedule(rack.setpointCommand(Intake.RackAndPinion.EXTEND));
        assertSame(Intake.RackAndPinion.EXTEND, rack.getSetpoint());
        runUntil(
            () -> rack.nearPosition(IntakeConstants.RackAndPinion.kOutAngle, IntakeConstants.RackAndPinion.converter.toAngle(Inches.of(0.5))),
            6.0,
            "Rack never extended, at " + IntakeConstants.RackAndPinion.converter.toDistance(rack.getPosition()).in(Inches) + " in");

        schedule(rack.setpointCommand(Intake.RackAndPinion.RETRACT));
        runUntil(
            () -> rack.nearPosition(IntakeConstants.RackAndPinion.kInAngle, IntakeConstants.RackAndPinion.converter.toAngle(Inches.of(0.5))),
            6.0,
            "Rack never retracted, at " + IntakeConstants.RackAndPinion.converter.toDistance(rack.getPosition()).in(Inches) + " in");
    }

    @Test
    void rollersReachIntakeSpeed() {
        Intake.Rollers rollers = Intake.Rollers.get();
        schedule(rollers.setpointCommand(Intake.Rollers.INTAKE));
        assertSame(Intake.Rollers.INTAKE, rollers.getSetpoint());

        runUntil(
            () -> Math.abs(rollers.getVelocity().in(RotationsPerSecond) - IntakeConstants.Roller.kIntakeSpeed.in(RotationsPerSecond)) < 2.0,
            4.0,
            "Intake rollers never reached intake speed, last read " + rollers.getVelocity().in(RotationsPerSecond));

        schedule(rollers.setpointCommand(Intake.Rollers.IDLE));
    }
}
