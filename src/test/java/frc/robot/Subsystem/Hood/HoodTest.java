package frc.robot.Subsystem.Hood;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.Subsystem.SubsystemTestBase;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

class HoodTest extends SubsystemTestBase {
    private static final double kEpsilon = 1e-9;

    @BeforeAll
    static void createSubsystems() {
        warmUp(Hood::get);
    }

    // ---- setpoints / constants ----

    @Test
    void stowAndMaxAreHoodLimits() {
        assertEquals(Mode.POSITIONPID, Hood.STOW.mode);
        assertEquals(Mode.POSITIONPID, Hood.MAX_ANGLE.mode);
        assertEquals(HoodConstants.kStowAngle.baseUnitMagnitude(), Hood.STOW.baseUnits, kEpsilon);
        assertEquals(HoodConstants.kMaxAngle.baseUnitMagnitude(), Hood.MAX_ANGLE.baseUnits, kEpsilon);
    }

    @Test
    void limitsAreSane() {
        assertTrue(HoodConstants.kMinAngle.in(Degrees) < HoodConstants.kMaxAngle.in(Degrees));
        assertTrue(HoodConstants.kEpsilonThreshold.in(Degrees) > 0.0);
    }

    @Test
    void homingConfigHomesToStowByDrivingDown() {
        var config = HoodConstants.getServoConfig();
        assertEquals(HoodConstants.kMinAngle.in(Degrees), config.kHomePosition.in(Degrees), kEpsilon);
        assertTrue(config.kHomingVoltage.baseUnitMagnitude() < 0.0, "homing should drive toward the min hard stop");
        assertTrue(config.kHomingTimeout.baseUnitMagnitude() > 0.0);
    }

    @Test
    void zeroConfigDrivesTowardTheHardStop() {
        var config = HoodConstants.getZeroConfig();
        assertTrue(config.kVoltage.baseUnitMagnitude() < 0.0, "zeroing should drive toward the bottom hard stop");
        assertEquals(0.0, config.kZeroPosition.in(Degrees), kEpsilon);
        assertTrue(HoodConstants.kStowAngle.in(Degrees) > config.kZeroPosition.in(Degrees), "stow should sit above the hard stop");
    }

    // ---- behavior in sim ----

    @Test
    void getReturnsSameInstance() {
        assertSame(Hood.get(), Hood.get());
    }

    @Test
    void movesToAngle() {
        Hood hood = Hood.get();
        Setpoint thirty = Setpoint.withPositionSetpoint(Degrees.of(30.0));
        schedule(hood.setpointCommand(thirty));
        assertSame(thirty, hood.getSetpoint());

        runUntil(
            () -> hood.nearPosition(Degrees.of(30.0), Degrees.of(2.0)),
            4.0,
            "Hood never reached 30 deg, at " + hood.getPosition().in(Degrees));
    }

    @Disabled("Stow is 2 deg now, not home (0 deg), so the lib's lazy homing never triggers; zeroing is covered by zeroingFindsTheHardStopThenStows")
    @Test
    void stowHomesAndFinishesInMotionMagic() {
        Hood hood = Hood.get();

        // move off home first so the homing sequence starts fresh
        schedule(hood.setpointCommand(Setpoint.withPositionSetpoint(Degrees.of(30.0))));
        runUntil(() -> hood.nearPosition(Degrees.of(30.0), Degrees.of(2.0)), 4.0, "Hood never left home");

        schedule(hood.setpointCommand(Hood.STOW));
        // homing kicks in once the hood is back near home: the lib switches to a voltage setpoint
        runUntil(() -> hood.getSetpoint().mode.isVoltageControl(), 4.0, "Hood never started homing");

        // homing ends by setting a motion magic setpoint at home
        runUntil(() -> hood.getSetpoint().mode == Mode.MOTIONMAGIC, 4.0, "Hood never finished homing");
        assertTrue(hood.nearPosition(HoodConstants.kMinAngle, Degrees.of(2.0)),
            "Hood should be at home after homing, at " + hood.getPosition().in(Degrees));
    }

    @Disabled("Stow is 2 deg now, not home (0 deg), so the lib's lazy homing never triggers; zeroing is covered by zeroingFindsTheHardStopThenStows")
    @Test
    void repeatedStowIsIgnoredWhileHoming() {
        Hood hood = Hood.get();

        schedule(hood.setpointCommand(Setpoint.withPositionSetpoint(Degrees.of(30.0))));
        runUntil(() -> hood.nearPosition(Degrees.of(30.0), Degrees.of(2.0)), 4.0, "Hood never left home");

        schedule(hood.setpointCommand(Hood.STOW));
        runUntil(() -> hood.getSetpoint().mode.isVoltageControl(), 4.0, "Hood never started homing");

        // this is what followSetpointCommand does every loop; Hood.applySetpoint must skip it
        hood.applySetpoint(Hood.STOW);
        assertTrue(hood.getSetpoint().mode.isVoltageControl(), "STOW while homing should not cancel homing");

        // a different target still goes through
        Setpoint twenty = Setpoint.withPositionSetpoint(Degrees.of(20.0));
        hood.applySetpoint(twenty);
        assertSame(twenty, hood.getSetpoint());
    }

    @Test
    void zeroingFindsTheHardStopThenStows() {
        Hood hood = Hood.get();

        // start away from the stop so it has to drive down
        schedule(hood.setpointCommand(Setpoint.withPositionSetpoint(Degrees.of(20.0))));
        runUntil(() -> hood.nearPosition(Degrees.of(20.0), Degrees.of(2.0)), 4.0, "Hood never left the stop");

        Command zero = hood.zeroAgainstHardStopCommand(HoodConstants.getZeroConfig(), Hood.STOW);
        schedule(zero);
        assertTrue(zero.isScheduled(), "Zeroing ended right away");
        assertTrue(hood.getSetpoint().mode.isVoltageControl(), "Zeroing should drive with voltage");
        runUntil(() -> !zero.isScheduled(), 8.0, "Hood zeroing never finished, at " + hood.getPosition().in(Degrees));

        assertTrue(hood.hasZeroed(), "Hood should be zeroed after the command finishes");
        assertSame(Hood.STOW, hood.getSetpoint());
        runUntil(() -> hood.nearPosition(HoodConstants.kStowAngle, Degrees.of(1.0)), 4.0,
            "Hood never moved off the hard stop to STOW, at " + hood.getPosition().in(Degrees));
    }

    @Test
    void zeroCommandIsSkippedOnceZeroed() {
        Hood hood = Hood.get();
        if (!hood.hasZeroed()) {
            Command zero = hood.zeroAgainstHardStopCommand(HoodConstants.getZeroConfig(), Hood.STOW);
            schedule(zero);
            runUntil(() -> !zero.isScheduled(), 8.0, "Hood zeroing never finished");
        }
        assertTrue(hood.hasZeroed());

        // a marker setpoint: a skipped zero must not touch the hood
        Setpoint marker = Setpoint.withPositionSetpoint(Degrees.of(10.0));
        hood.applySetpoint(marker);
        Command again = hood.zeroCommand();
        schedule(again);
        runFor(0.1);
        assertFalse(again.isScheduled(), "Zeroing should end right away once already zeroed");
        assertSame(marker, hood.getSetpoint());
    }
}
