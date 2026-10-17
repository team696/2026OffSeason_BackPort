package frc.lib.io.vision.limelight;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.LimelightHelpers;
import frc.lib.util.vision.LimelightHelpers.PoseEstimate;
import frc.lib.util.vision.LimelightHelpers.RawDetection;
import frc.lib.util.vision.VisionEstimate;
import frc.lib.util.vision.VisionGamePiece;
import frc.lib.util.vision.VisionTelemetry;
import frc.lib.util.vision.VisionTelemetry.Decision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CombinedLimelightCameraIO extends LimelightCameraIO {

	private static final double MAX_VISION_YAW_RATE_DEG_PER_SEC = 720.0;

	private final VisionTelemetry diagnostics;

	public CombinedLimelightCameraIO(CameraIOConfig config) {
		super(config);
		diagnostics = new VisionTelemetry(NetworkTableInstance.getDefault().getTable("Vision").getSubTable(config.name));
	}

	public Translation2d calcDistanceToGamePiece(double tx, double ty, Distance radius) {

		Angle totalAngleX =
				Degrees.of(-ty).minus(config.robotToCameraOffset.getRotation().getMeasureY());

		Distance distAwayX =
				config.robotToCameraOffset.getMeasureZ().minus(radius).div(Math.tan(totalAngleX.in(Units.Radians)));

		Distance distHypotenuseYToGround = BaseUnits.DistanceUnit.of(Math.hypot(
				distAwayX.in(BaseUnits.DistanceUnit),
				config.robotToCameraOffset.getMeasureZ().minus(radius).in(BaseUnits.DistanceUnit)));

		Angle totalAngleY = Units.Degrees.of(-tx)
				.plus(config.robotToCameraOffset.getRotation().getMeasureZ());

		Distance distAwayY = distHypotenuseYToGround.times(Math.tan(totalAngleY.in(Units.Radians))); // robot y

		return new Translation2d(distAwayX, distAwayY);
	}

	private double getYawRate() {
		return Swerve.get().getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
	}

	private void updateGyro() {
		LimelightHelpers.SetRobotOrientation_NoFlush(
				config.name, Drive.mInstance.getPose().getRotation().getDegrees(), getYawRate(), 0, 0, 0, 0);
	}

	// spotless:off

	private static boolean isValidEstimate(PoseEstimate estimate) {
		if (estimate == null || estimate.pose == null || estimate.tagCount < 1) {
			return false;
		}
		// Reject the default origin pose without rejecting valid poses on either field axis.
		return !(estimate.pose.getX() == 0.0
				&& estimate.pose.getY() == 0.0
				&& estimate.pose.getRotation().getRadians() == 0.0);
	}

	@Override
	public Optional<List<VisionEstimate>> getLastEstimates() {
		boolean disabled = DriverStation.isDisabled();
		double yawRate = getYawRate();
		PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
		diagnostics.observe(limelightEstimate, Drive.mInstance.getPose(), Timer.getFPGATimestamp(), disabled);
		// Disabled: MegaTag1 seeds the full pose, including heading.
		if (disabled) {
			if (isValidEstimate(limelightEstimate)) {
				Swerve.get().resetPoseWithReason(limelightEstimate.pose, "DisabledVision:" + config.name);
				diagnostics.decision(Decision.DISABLED_POSE_RESET, Drive.mInstance.getPose(), new double[0]);
			} else {
				logInvalidEstimate(limelightEstimate);
			}
			return Optional.empty();
		}

		// Enabled: MegaTag1 for x/y; heading stays on the Pigeon through fusion standard deviations.
		if (Math.abs(yawRate) > MAX_VISION_YAW_RATE_DEG_PER_SEC) {
			diagnostics.decision(Decision.HIGH_YAW_RATE, Drive.mInstance.getPose(), new double[0]);
			return Optional.empty();
		}
		if (!isValidEstimate(limelightEstimate)) {
			logInvalidEstimate(limelightEstimate);
			return Optional.empty();
		}
		int buffer[] = new int[limelightEstimate.rawFiducials.length];
		for (int i = 0; i < buffer.length; i++) buffer[i] = limelightEstimate.rawFiducials[i].id;
		VisionEstimate estimate = new VisionEstimate(
						limelightEstimate.pose,
						Seconds.of(limelightEstimate.timestampSeconds),
						FieldLayout.getAprilTagArrayFromIDs(buffer))
				.withAverageDistance(Meters.of(limelightEstimate.avgTagDist));
		return Optional.of(List.of(estimate));
	}

	private void logInvalidEstimate(PoseEstimate estimate) {
		Decision reason = estimate == null ? Decision.NULL_ESTIMATE
			: estimate.pose == null ? Decision.NULL_POSE
			: estimate.tagCount < 1 ? Decision.NO_TAGS : Decision.ORIGIN_POSE;
		diagnostics.decision(reason, Drive.mInstance.getPose(), new double[0]);
	}

	@Override
	public void logLocalizationSkipped(String reason) {
		diagnostics.observe(LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name), Drive.mInstance.getPose(),
			Timer.getFPGATimestamp(), DriverStation.isDisabled());
		diagnostics.decision(Decision.valueOf(reason), Drive.mInstance.getPose(), new double[0]);
		logGyro();
	}

	@Override
	public void logVisionSubmitted(VisionEstimate estimate, Vector<N3> deviations, Pose2d poseAfter) {
		diagnostics.decision(Decision.FUSION_SUBMITTED, poseAfter,
			new double[] {deviations.get(0), deviations.get(1), deviations.get(2)});
	}

	@Override
	public Optional<List<VisionGamePiece>> getAllGamePieces() {
		ArrayList<VisionGamePiece> all = new ArrayList<>();
		for (RawDetection raw : LimelightHelpers.getRawDetections(config.name)) {
			if (raw.classId == 0)
				continue;
			double tx = raw.txnc, ty = raw.tync;
			Translation2d coralTranslation = calcDistanceToGamePiece(tx, ty, Units.Inches.of(4.5 / 2))
					.minus(config.robotToCameraOffset.getTranslation().toTranslation2d());
			Pose2d coralPose =
					Drive.mInstance.getPose().transformBy(new Transform2d(coralTranslation, new Rotation2d()));
			Translation2d t = coralPose.getTranslation();
			if (FieldLayout.outsideField(coralPose)) {
				continue;
			}
			// all.add(new VisionGamePiece(
			// new Pose2d(
			// t,
			// t.minus(Drive.mInstance.getPose().getTranslation()).getAngle()),
			// Seconds.of(Timer.getFPGATimestamp())));
			all.add(new VisionGamePiece(coralPose));
		}
		return Optional.ofNullable(all);
	}

	// spotless:on

	@Override
	public void update() {
		updateGyro();
		logGyro();
	}

	private void logGyro() {
		var pigeon = Swerve.get().getPigeon2();
		var yaw = pigeon.getYaw();
		var pitch = pigeon.getPitch();
		diagnostics.gyro(yaw.getValueAsDouble(), pitch.getValueAsDouble(), getYawRate(),
			yaw.getTimestamp().getLatency(), pitch.getTimestamp().getLatency());
	}
}
