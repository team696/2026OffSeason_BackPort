package frc.robot.Subsystem.Hopper;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.io.MotorIO.Mode;
import frc.robot.Subsystem.SubsystemTestBase;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class HopperPivotTest extends SubsystemTestBase {
    private static final double kEpsilon = 1e-9;
    private static final double kToleranceDeg = 1.0;

    @BeforeAll
    static void createSubsystems() {
        warmUp(Hopper.Pivot::get);
    }

    // ---- setpoints ----

    @Test
    void setpointsArePositionControl() {
        assertEquals(Mode.POSITIONPID, Hopper.Pivot.UP.mode);
        assertEquals(Mode.POSITIONPID, Hopper.Pivot.ZERO.mode);
    }

    @Test
    void setpointsMatchConstants() {
        assertEquals(HopperConstants.HopperPivot.kHighest.baseUnitMagnitude(), Hopper.Pivot.UP.baseUnits, kEpsilon);
        assertEquals(HopperConstants.HopperPivot.kLowestAngle.baseUnitMagnitude(), Hopper.Pivot.ZERO.baseUnits, kEpsilon);
    }

    @Test
    void upIsAboveDown() {
        assertTrue(HopperConstants.HopperPivot.kHighest.gt(HopperConstants.HopperPivot.kLowestAngle));
    }

    // ---- behavior in sim ----

    @Test
    void getReturnsSameInstance() {
        assertSame(Hopper.Pivot.get(), Hopper.Pivot.get());
    }

    @Test
    void startsDown() {
        Hopper.Pivot pivot = Hopper.Pivot.get();
        schedule(pivot.setpointCommand(Hopper.Pivot.ZERO));
        runFor(0.5);
        assertTrue(pivot.nearPosition(HopperConstants.HopperPivot.kLowestAngle, Degrees.of(kToleranceDeg)),
            "Pivot should start at the bottom, at " + pivot.getPosition().in(Degrees) + " deg");
    }

    @Test
    void goesUpAndComesBackDown() {
        Hopper.Pivot pivot = Hopper.Pivot.get();

        schedule(pivot.setpointCommand(Hopper.Pivot.UP));
        assertSame(Hopper.Pivot.UP, pivot.getSetpoint());
        runUntil(() -> pivot.nearPosition(HopperConstants.HopperPivot.kHighest, Degrees.of(kToleranceDeg)), 5.0,
            "Pivot never reached UP (" + HopperConstants.HopperPivot.kHighest.in(Degrees) + " deg)");

        schedule(pivot.setpointCommand(Hopper.Pivot.ZERO));
        assertSame(Hopper.Pivot.ZERO, pivot.getSetpoint());
        runUntil(() -> pivot.nearPosition(HopperConstants.HopperPivot.kLowestAngle, Degrees.of(kToleranceDeg)), 5.0,
            "Pivot never came back DOWN (" + HopperConstants.HopperPivot.kLowestAngle.in(Degrees) + " deg)");
    }

    @Test
    void testAngleFollowsDashboard() {
        Hopper.Pivot pivot = Hopper.Pivot.get();
        SmartDashboard.putNumber("TunableNumbers/Hopper Pivot Test Angle", 5.0);
        schedule(pivot.getTestAngleCommand());
        runUntil(() -> pivot.nearPosition(Degrees.of(5.0), Degrees.of(kToleranceDeg)), 5.0, "Pivot never reached Test Angle (5 deg)");

        SmartDashboard.putNumber("TunableNumbers/Hopper Pivot Test Angle", 2.0);
        runUntil(() -> pivot.nearPosition(Degrees.of(2.0), Degrees.of(kToleranceDeg)), 5.0,
            "Pivot didn't follow the new Test Angle (2 deg)");

        SmartDashboard.putNumber("TunableNumbers/Hopper Pivot Test Angle", 0.0);
        schedule(pivot.setpointCommand(Hopper.Pivot.ZERO));
        runFor(2.0);
    }

    @Test
    void stopIsNeutral() {
        Hopper.Pivot pivot = Hopper.Pivot.get();
        schedule(pivot.setpointCommand(Hopper.Pivot.STOP));
        assertSame(Hopper.Pivot.STOP, pivot.getSetpoint());
        assertEquals(Mode.IDLE, Hopper.Pivot.STOP.mode);
    }

    @Test
    void holdsUp() {
        Hopper.Pivot pivot = Hopper.Pivot.get();
        schedule(pivot.setpointCommand(Hopper.Pivot.UP));
        runUntil(() -> pivot.nearPosition(HopperConstants.HopperPivot.kHighest, Degrees.of(kToleranceDeg)), 5.0,
            "Pivot never reached UP");
        runFor(1.0);
        assertTrue(pivot.nearPosition(HopperConstants.HopperPivot.kHighest, Degrees.of(kToleranceDeg)),
            "Pivot drifted off UP, at " + pivot.getPosition().in(Degrees) + " deg");
        schedule(pivot.setpointCommand(Hopper.Pivot.ZERO));
        runFor(2.0);
    }
}
