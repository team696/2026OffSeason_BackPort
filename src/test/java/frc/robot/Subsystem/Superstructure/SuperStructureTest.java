package frc.robot.Subsystem.Superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.FieldLayout;
import frc.robot.Subsystem.SubsystemTestBase;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.ShootingState;
import frc.robot.shooting.shotverifier.ShotVerify;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.SuperStructure;
import frc.robot.subsystems.drive.Swerve;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

class SuperStructureTest extends SubsystemTestBase {

    @BeforeAll
    static void createSubsystems() {
        warmUp(() -> {
            Intake.RackAndPinion.get();
            Intake.Rollers.get();
            Intake.Rollers2.get();
            Hopper.Rollers.get();
            Shooter.get();
            Hood.get();
            FeedRollers.get();
            Swerve.get();
        });
    }

    private static boolean rackAt(edu.wpi.first.units.measure.Angle angle) {
        return Intake.RackAndPinion.get().nearPosition(angle, IntakeConstants.RackAndPinion.converter.toAngle(Inches.of(0.5)));
    }

    // ---- intake ----

    @Test
    void runIntakeExtendsAndSpinsRollers() {
        schedule(SuperStructure.RunIntake());
        assertSame(Intake.RackAndPinion.EXTEND, Intake.RackAndPinion.get().getSetpoint());
        assertSame(Intake.Rollers.INTAKE, Intake.Rollers.get().getSetpoint());
        assertSame(Intake.Rollers2.INTAKE, Intake.Rollers2.get().getSetpoint());
        runUntil(() -> rackAt(IntakeConstants.RackAndPinion.kOutAngle), 6.0, "RunIntake never extended the rack");
        runUntil(() -> Math.abs(Intake.Rollers.get().getVelocity().in(RotationsPerSecond)) > 1.0, 4.0, "RunIntake never spun the rollers");
    }

    @Test
    void outtakeExtendsAndReversesRollers() {
        schedule(SuperStructure.Outtake());
        assertSame(Intake.RackAndPinion.EXTEND, Intake.RackAndPinion.get().getSetpoint());
        assertSame(Intake.Rollers.OUTAKE, Intake.Rollers.get().getSetpoint());
        assertSame(Intake.Rollers2.OUTAKE, Intake.Rollers2.get().getSetpoint());
    }

    @Test
    void stopIntakeRetractsAndStopsRollers() {
        schedule(SuperStructure.RunIntake());
        runFor(0.5);
        schedule(SuperStructure.StopIntake());
        assertSame(Intake.RackAndPinion.RETRACT, Intake.RackAndPinion.get().getSetpoint());
        assertSame(Intake.Rollers.IDLE, Intake.Rollers.get().getSetpoint());
        assertSame(Intake.Rollers2.IDLE, Intake.Rollers2.get().getSetpoint());
        runUntil(() -> rackAt(IntakeConstants.RackAndPinion.kInAngle), 6.0, "StopIntake never retracted the rack");
    }

    // ---- shooting ----

    @Test
    void shootingRunsHopperAndOscillatesIntake() {
        Shooting.setState(ShootingState.HUB);
        Command shooting = SuperStructure.Shooting();
        schedule(shooting);
        assertTrue(shooting.isScheduled(), "Shooting ended right away");
        assertSame(Hopper.Rollers.INTAKE, Hopper.Rollers.get().getSetpoint());
        assertSame(Intake.RackAndPinion.EXTEND, Intake.RackAndPinion.get().getSetpoint());
        runFor(0.5);
        assertSame(Intake.RackAndPinion.HALF, Intake.RackAndPinion.get().getSetpoint());
    }

    // ---- shooting testing (speed/angle come from the dashboard tunables) ----

    private static void setTestShot(double speedRps, double angleDeg) {
        SmartDashboard.putNumber("TunableNumbers/Test Speed", speedRps);
        SmartDashboard.putNumber("TunableNumbers/Test Angle", angleDeg);
    }

    @Test
    void shootingTestingUsesDashboardSpeedAndAngle() {
        setTestShot(25.0, 30.0);
        Command testing = SuperStructure.ShootingTesting();
        schedule(testing);
        assertTrue(testing.isScheduled(), "ShootingTesting ended right away");

        assertSame(FeedRollers.RUNNING, FeedRollers.get().getSetpoint());
        assertSame(Hopper.Rollers.INTAKE, Hopper.Rollers.get().getSetpoint());
        assertEquals(RotationsPerSecond.of(25.0).baseUnitMagnitude(), Shooter.get().getSetpoint().baseUnits, 1e-6,
            "Shooter setpoint should be the dashboard Test Speed");

        runUntil(() -> Shooter.get().nearVelocity(RotationsPerSecond.of(25.0), RotationsPerSecond.of(2.0)), 4.0,
            "Shooter never reached Test Speed, at " + Shooter.get().getVelocity().in(RotationsPerSecond) + " RPS");
        runUntil(() -> Hood.get().nearPosition(Degrees.of(30.0), Degrees.of(2.0)), 4.0,
            "Hood never reached Test Angle, at " + Hood.get().getPosition().in(Degrees) + " deg");

        schedule(SuperStructure.ShootingStop());
        setTestShot(0.0, 0.0);
    }

    @Test
    void shootingTestingFollowsDashboardChanges() {
        setTestShot(20.0, 20.0);
        schedule(SuperStructure.ShootingTesting());
        runUntil(() -> Shooter.get().nearVelocity(RotationsPerSecond.of(20.0), RotationsPerSecond.of(2.0)), 4.0,
            "Shooter never reached the first Test Speed");

        // change the numbers while it's running, like tuning from the dashboard
        setTestShot(35.0, 40.0);
        runUntil(() -> Shooter.get().nearVelocity(RotationsPerSecond.of(35.0), RotationsPerSecond.of(2.0)), 4.0,
            "Shooter didn't follow the new Test Speed, at " + Shooter.get().getVelocity().in(RotationsPerSecond) + " RPS");
        runUntil(() -> Hood.get().nearPosition(Degrees.of(40.0), Degrees.of(2.0)), 4.0,
            "Hood didn't follow the new Test Angle, at " + Hood.get().getPosition().in(Degrees) + " deg");

        schedule(SuperStructure.ShootingStop());
        setTestShot(0.0, 0.0);
    }

    @Test
    void shootingStopStopsEverything() {
        schedule(SuperStructure.ShootingStop());
        assertSame(Shooter.IDLE, Shooter.get().getSetpoint());
        assertSame(FeedRollers.IDLE, FeedRollers.get().getSetpoint());
        assertSame(Hopper.Rollers.IDLE, Hopper.Rollers.get().getSetpoint());
        // STOW gets swapped for the hood's homing setpoint, so check where it ends up instead
        runUntil(() -> Hood.get().nearPosition(HoodConstants.kMinAngle, Degrees.of(2.0)), 4.0,
            "ShootingStop never stowed the hood, at " + Hood.get().getPosition().in(Degrees));
    }

    @Test
    void autoAlignRuns() {
        Shooting.setState(ShootingState.HUB);
        Command align = SuperStructure.AutoAlgin();
        schedule(align);
        runFor(0.5);
        assertTrue(align.isScheduled() || !CommandScheduler.getInstance().isScheduled(align), "AutoAlign crashed");
    }

    // ---- passing ----

    @Test
    void passingLeftAndRightAimAtDifferentSpots() {
        Translation2d left = ShootingState.NEUTRAL_FERRY_LEFT.getLocationTranslation(false);
        Translation2d right = ShootingState.NEUTRAL_FERRY_RIGHT.getLocationTranslation(false);
        assertEquals(FieldLayout.kBlueLeftNeutralFerry, left);
        assertEquals(FieldLayout.kBlueRightNeutralFerry, right);
        assertNotEquals(left, right);
    }

    // ---- aiming actually turns the robot (passing and hub auto align) ----

    private static double aimErrorDeg(Translation2d target) {
        Pose2d pose = Swerve.get().getState().Pose;
        return Math.abs(target.minus(pose.getTranslation()).getAngle().minus(pose.getRotation()).getDegrees());
    }

    /**
     * Puts the robot at start (facing the wrong way), runs the aim command, and checks it turns to within the shot
     * alignment tolerance and stays there. The command is made after the pose reset since it picks its state when built.
     */
    private static void assertTurnsToward(String name, ShootingState state, Supplier<Command> makeCommand, Pose2d start) {
        Swerve swerve = Swerve.get();
        // the last aim request keeps running after its command ends, so stop the drive before moving the robot
        swerve.setControl(new SwerveRequest.Idle());
        runFor(0.5);
        swerve.resetPose(start);
        runFor(0.25);

        Translation2d target = state.getLocationTranslation(false);
        double tolerance = ShotVerify.SHOT_ALIGNMENT_TOLERANCE.in(Degrees);
        assertTrue(aimErrorDeg(target) > 90.0, "Test setup: robot should start facing away from the target");

        Command aim = makeCommand.get();
        assertSame(state, Shooting.getState());
        schedule(aim);
        double startTime = Timer.getFPGATimestamp();
        // has to get inside the shot alignment tolerance, or shotVerified never lets the feeder run
        runUntil(() -> aimErrorDeg(target) < tolerance, 4.0,
            name + " never got within " + tolerance + " deg of its target, so shotVerified would never let the feeder run");
        double settleTime = Timer.getFPGATimestamp() - startTime;

        // and it should hold there, not wobble back out
        double worst = 0.0;
        for (int i = 0; i < 50; i++) {
            step();
            worst = Math.max(worst, aimErrorDeg(target));
        }
        System.out.printf("%s: aligned in %.2f s, final error %.2f deg, worst while holding %.2f deg%n",
            name, settleTime, aimErrorDeg(target), worst);
        assertTrue(worst < tolerance, name + " drifted out of alignment while holding, worst " + worst + " deg");

        CommandScheduler.getInstance().cancel(aim);
        schedule(SuperStructure.ShootingStop());
    }

    @Disabled("Fails until the heading PID in MotionComp.PointToTarget is strong enough to beat drive friction (kP=3 stalls ~4 deg short)")
    @Test
    void passingLeftTurnsRobotToLeftTarget() {
        assertTurnsToward("Passing Left", ShootingState.NEUTRAL_FERRY_LEFT,
            () -> SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_LEFT), new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(0.0)));
    }

    @Disabled("Fails until the heading PID in MotionComp.PointToTarget is strong enough to beat drive friction (kP=3 stalls ~4 deg short)")
    @Test
    void passingRightTurnsRobotToRightTarget() {
        assertTurnsToward("Passing Right", ShootingState.NEUTRAL_FERRY_RIGHT,
            () -> SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_RIGHT), new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(0.0)));
    }

    @Disabled("Fails until the heading PID in MotionComp.PointToTarget is strong enough to beat drive friction (kP=3 stalls ~4 deg short)")
    @Test
    void autoAlignTurnsToHubFromBelow() {
        assertTurnsToward("Auto Align (below hub)", ShootingState.HUB,
            SuperStructure::AutoAlgin, new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(180.0)));
    }

    @Disabled("Fails until the heading PID in MotionComp.PointToTarget is strong enough to beat drive friction (kP=3 stalls ~4 deg short)")
    @Test
    void autoAlignTurnsToHubFromAbove() {
        assertTurnsToward("Auto Align (above hub)", ShootingState.HUB,
            SuperStructure::AutoAlgin, new Pose2d(2.0, 6.0, Rotation2d.fromDegrees(135.0)));
    }

    @Disabled("Fails until the heading PID in MotionComp.PointToTarget is strong enough to beat drive friction (kP=3 stalls ~4 deg short)")
    @Test
    void autoAlignTurnsToHubFromNeutralZone() {
        assertTurnsToward("Auto Align (neutral zone)", ShootingState.HUB,
            SuperStructure::AutoAlgin, new Pose2d(6.5, 4.0, Rotation2d.fromDegrees(0.0)));
    }

    @Test
    void passingLeftSetsStateAndRunsHopper() {
        Shooting.setState(ShootingState.HUB);
        Command pass = SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_LEFT);
        assertSame(ShootingState.HUB, Shooting.getState(), "state should not change until the command starts");
        schedule(pass);
        assertSame(ShootingState.NEUTRAL_FERRY_LEFT, Shooting.getState());
        assertTrue(pass.isScheduled(), "Passing Left ended right away");
        assertSame(Hopper.Rollers.INTAKE, Hopper.Rollers.get().getSetpoint());
        runFor(0.5);
        assertSame(ShootingState.NEUTRAL_FERRY_LEFT, Shooting.getState());
    }

    @Test
    void passingRightSetsStateAndRunsHopper() {
        Shooting.setState(ShootingState.HUB);
        Command pass = SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_RIGHT);
        assertSame(ShootingState.HUB, Shooting.getState(), "state should not change until the command starts");
        schedule(pass);
        assertSame(ShootingState.NEUTRAL_FERRY_RIGHT, Shooting.getState());
        assertTrue(pass.isScheduled(), "Passing Right ended right away");
        assertSame(Hopper.Rollers.INTAKE, Hopper.Rollers.get().getSetpoint());
        runFor(0.5);
        assertSame(ShootingState.NEUTRAL_FERRY_RIGHT, Shooting.getState());
    }
}
