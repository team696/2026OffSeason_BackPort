package frc.robot.Subsystem.Intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Subsystem.SubsystemTestBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Superstructure.SuperStructureConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class IntakeTest extends SubsystemTestBase {
    private static final double kEpsilon = 1e-9;

    @BeforeAll
    static void createSubsystems() {
        warmUp(() -> {
            Intake.RackAndPinion.get();
            Intake.Rollers.get();
            Intake.Rollers2.get();
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
        assertEquals(IntakeConstants.RackAndPinion.kHalfAngle.baseUnitMagnitude(), Intake.RackAndPinion.HALF.baseUnits, kEpsilon);
    }

    @Test
    void rackSetpointsAreOrdered() {
        assertTrue(Intake.RackAndPinion.RETRACT.baseUnits < Intake.RackAndPinion.HALF.baseUnits);
        assertTrue(Intake.RackAndPinion.HALF.baseUnits < Intake.RackAndPinion.EXTEND.baseUnits);
    }

    @Test
    void angleDistanceConversionRoundTrips() {
        var converter = IntakeConstants.RackAndPinion.converter;
        assertEquals(IntakeConstants.RackAndPinion.kOutDistance.in(Inches), converter.toDistance(IntakeConstants.RackAndPinion.kOutAngle).in(Inches), 1e-6);
        assertEquals(IntakeConstants.RackAndPinion.kInDistance.in(Inches), converter.toDistance(IntakeConstants.RackAndPinion.kInAngle).in(Inches), 1e-6);
        assertEquals(IntakeConstants.RackAndPinion.kHalfDistance.in(Inches), converter.toDistance(IntakeConstants.RackAndPinion.kHalfAngle).in(Inches), 1e-6);
    }

    @Test
    void hardStopIsBelowEveryTarget() {
        assertTrue(IntakeConstants.RackAndPinion.kHardStopAngle.baseUnitMagnitude() < Intake.RackAndPinion.RETRACT.baseUnits);
        assertEquals(0.0, IntakeConstants.RackAndPinion.kHardStopDistance.in(Inches), kEpsilon);
    }

    @Test
    void zeroConfigDrivesTowardTheHardStop() {
        var config = IntakeConstants.RackAndPinion.getZeroConfig();
        assertTrue(config.kVoltage.baseUnitMagnitude() < 0.0, "zeroing should drive toward the retracted hard stop");
        assertEquals(IntakeConstants.RackAndPinion.kHardStopAngle.baseUnitMagnitude(), config.kZeroPosition.baseUnitMagnitude(), kEpsilon);
        assertTrue(config.kTimeout.baseUnitMagnitude() > config.kStartupGrace.baseUnitMagnitude() + config.kStalledTime.baseUnitMagnitude());
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

    @Test
    void roller2SetpointsUseRoller2Constants() {
        assertEquals(Mode.VELOCITY, Intake.Rollers2.INTAKE.mode);
        assertEquals(Mode.VELOCITY, Intake.Rollers2.OUTAKE.mode);
        assertEquals(IntakeConstants.Roller2.kIntakeSpeed.baseUnitMagnitude(), Intake.Rollers2.INTAKE.baseUnits, kEpsilon);
        assertEquals(IntakeConstants.Roller2.kOuttakeSpeed.baseUnitMagnitude(), Intake.Rollers2.OUTAKE.baseUnits, kEpsilon);
        assertSame(Setpoint.COAST, Intake.Rollers2.IDLE);
    }

    @Test
    void rollersAreSeparateMotors() {
        assertTrue(IntakeConstants.Roller.getIOConfig().mainID != IntakeConstants.Roller2.getIOConfig().mainID);
    }

    // ---- behavior in sim ----

    @Test
    void getReturnsSameInstance() {
        assertSame(Intake.RackAndPinion.get(), Intake.RackAndPinion.get());
        assertSame(Intake.Rollers.get(), Intake.Rollers.get());
        assertSame(Intake.Rollers2.get(), Intake.Rollers2.get());
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
    void zeroingFindsTheHardStopThenRetracts() {
        Intake.RackAndPinion rack = Intake.RackAndPinion.get();
        var tolerance = IntakeConstants.RackAndPinion.converter.toAngle(Inches.of(0.5));

        // start away from the stop so it has to drive in
        schedule(rack.setpointCommand(Intake.RackAndPinion.EXTEND));
        runUntil(() -> rack.nearPosition(IntakeConstants.RackAndPinion.kOutAngle, tolerance), 6.0, "Rack never extended");

        Command zero = rack.zeroAgainstHardStopCommand(IntakeConstants.RackAndPinion.getZeroConfig(), Intake.RackAndPinion.RETRACT);
        schedule(zero);
        assertTrue(zero.isScheduled(), "Zeroing ended right away");
        assertTrue(rack.getSetpoint().mode.isVoltageControl(), "Zeroing should drive with voltage");
        runUntil(() -> !zero.isScheduled(), 12.0, "Rack zeroing never finished");

        assertTrue(rack.hasZeroed(), "Rack should be zeroed after the command finishes");
        assertSame(Intake.RackAndPinion.RETRACT, rack.getSetpoint());
        runUntil(() -> rack.nearPosition(IntakeConstants.RackAndPinion.kInAngle, tolerance), 6.0,
            "Rack never moved off the hard stop to RETRACT, at " + IntakeConstants.RackAndPinion.converter.toDistance(rack.getPosition()).in(Inches) + " in");
    }

    @Test
    void zeroCommandIsSkippedOnceZeroed() {
        Intake.RackAndPinion rack = Intake.RackAndPinion.get();
        if (!rack.hasZeroed()) {
            Command zero = rack.zeroAgainstHardStopCommand(IntakeConstants.RackAndPinion.getZeroConfig(), Intake.RackAndPinion.RETRACT);
            schedule(zero);
            runUntil(() -> !zero.isScheduled(), 12.0, "Rack zeroing never finished");
        }
        assertTrue(rack.hasZeroed());

        // a marker setpoint: a skipped zero must not touch the rack
        rack.applySetpoint(Intake.RackAndPinion.EXTEND);
        Command again = rack.zeroCommand();
        schedule(again);
        runFor(0.1);
        assertFalse(again.isScheduled(), "Zeroing should end right away once already zeroed");
        assertSame(Intake.RackAndPinion.EXTEND, rack.getSetpoint());
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

    // the intake/outtake/stop groups crashed robotInit when both halves required Rollers
    @Test
    void rollerGroupsDriveBothRollers() {
        schedule(SuperStructureConstants.IntakeRollers());
        assertSame(Intake.Rollers.INTAKE, Intake.Rollers.get().getSetpoint());
        assertSame(Intake.Rollers2.INTAKE, Intake.Rollers2.get().getSetpoint());

        schedule(SuperStructureConstants.OutakeRollers());
        assertSame(Intake.Rollers.OUTAKE, Intake.Rollers.get().getSetpoint());
        assertSame(Intake.Rollers2.OUTAKE, Intake.Rollers2.get().getSetpoint());

        schedule(SuperStructureConstants.StopIntakeRollers());
        assertSame(Intake.Rollers.IDLE, Intake.Rollers.get().getSetpoint());
        assertSame(Intake.Rollers2.IDLE, Intake.Rollers2.get().getSetpoint());
    }
}
