package frc.lib.io.vision.limelight;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.LimelightHelpers;
import frc.lib.util.vision.LimelightHelpers.PoseEstimate;
import frc.lib.util.vision.VisionEstimate;
import frc.lib.util.vision.VisionTelemetry;
import frc.lib.util.vision.VisionTelemetry.Decision;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTagLimelightCameraIO extends LimelightCameraIO {

	private static final double MAX_TAG_DISTANCE_METERS = 2.5;
	private static final double MAX_TAG_AMBIGUITY = 0.43;
	private static final double MAX_FRAME_AGE_SECONDS = 0.25;
	private static final double MAX_FUTURE_TIMESTAMP_SECONDS = 0.05;
	private final VisionTelemetry diagnostics;
	private double lastProcessedTimestamp = Double.NEGATIVE_INFINITY;

	private Optional<VisionEstimate> lastGivenEstimate = Optional.empty();

	public AprilTagLimelightCameraIO(CameraIOConfig config) {
		super(config);
		diagnostics = new VisionTelemetry(NetworkTableInstance.getDefault().getTable("Vision").getSubTable(config.name));
	}

	private void updateGyro() {
		LimelightHelpers.SetRobotOrientation_NoFlush(
				config.name, Drive.mInstance.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
	}

	// spotless:off

	static boolean isValidEstimate(PoseEstimate estimate) {
		if (estimate == null || estimate.pose == null || estimate.tagCount < 1
				|| estimate.rawFiducials == null || estimate.rawFiducials.length != estimate.tagCount) {
			return false;
		}
		double x = estimate.pose.getX();
		double y = estimate.pose.getY();
		if (!Double.isFinite(x) || !Double.isFinite(y)
				|| !Double.isFinite(estimate.pose.getRotation().getRadians())
				|| (x == 0.0 && y == 0.0)
				|| x < 0.0 || x > FieldLayout.kFieldLength.in(Meters)
				|| y < 0.0 || y > FieldLayout.kFieldWidth.in(Meters)
				|| !Double.isFinite(estimate.avgTagDist)
				|| estimate.avgTagDist < 0.0 || estimate.avgTagDist > MAX_TAG_DISTANCE_METERS) {
			return false;
		}
		// Check every contributing tag so a nearby tag cannot hide a distant or ambiguous one.
		for (var tag : estimate.rawFiducials) {
			if (tag == null || !Double.isFinite(tag.distToCamera)
					|| tag.distToCamera < 0.0 || tag.distToCamera > MAX_TAG_DISTANCE_METERS
					|| !Double.isFinite(tag.ambiguity)
					|| tag.ambiguity < 0.0 || tag.ambiguity > MAX_TAG_AMBIGUITY) {
				return false;
			}
		}
		return true;
	}

	@Override
	public Optional<VisionEstimate> getRobotPose() {
		updateGyro();
		PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
		if (!isValidEstimate(limelightEstimate))
			return (lastGivenEstimate = Optional.empty());

		int buffer[] = new int[limelightEstimate.rawFiducials.length];
		for (int i = 0; i < buffer.length; i++)
			buffer[i] = limelightEstimate.rawFiducials[i].id;
		lastGivenEstimate = Optional.of(new VisionEstimate(
						limelightEstimate.pose,
						Seconds.of(limelightEstimate.timestampSeconds),
						FieldLayout.getAprilTagArrayFromIDs(buffer))
				.withAverageDistance(Meters.of(limelightEstimate.avgTagDist)));
		return lastGivenEstimate;
	}

	// spotless:on

	@Override
	public Optional<List<VisionEstimate>> getLastEstimates() {
		updateGyro();
		PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);

		double now = Timer.getFPGATimestamp();
		diagnostics.observe(limelightEstimate, Drive.mInstance.getPose(), now, DriverStation.isDisabled());
		if (limelightEstimate != null) {
			double timestamp = limelightEstimate.timestampSeconds;
			if (!Double.isFinite(timestamp) || now - timestamp > MAX_FRAME_AGE_SECONDS
					|| timestamp - now > MAX_FUTURE_TIMESTAMP_SECONDS) {
				diagnostics.decision(Decision.INVALID_FRAME_TIME, Drive.mInstance.getPose(), new double[0]);
				return Optional.empty();
			}
			if (timestamp <= lastProcessedTimestamp) {
				diagnostics.decision(Decision.DUPLICATE_OR_OUT_OF_ORDER, Drive.mInstance.getPose(), new double[0]);
				return Optional.empty();
			}
			// Consume each image once, even when its quality checks fail.
			lastProcessedTimestamp = timestamp;
		}

		  if (limelightEstimate != null && limelightEstimate.rawFiducials != null
				&& limelightEstimate.rawFiducials.length > 0 && limelightEstimate.rawFiducials[0] != null) {
        SmartDashboard.putNumber("Debug/Ambiguity", limelightEstimate.rawFiducials[0].ambiguity);
    		}
    	SmartDashboard.putNumber("Debug/TagCount", limelightEstimate == null ? -1 : limelightEstimate.tagCount);
		if (!isValidEstimate(limelightEstimate)) {
			diagnostics.decision(Decision.INVALID_ESTIMATE, Drive.mInstance.getPose(), new double[0]);
			return Optional.empty();
		}
		int buffer[] = new int[limelightEstimate.rawFiducials.length];
		for (int i = 0; i < buffer.length; i++) buffer[i] = limelightEstimate.rawFiducials[i].id;
		VisionEstimate estimate = new VisionEstimate(
						limelightEstimate.pose,
						Seconds.of(limelightEstimate.timestampSeconds),
						FieldLayout.getAprilTagArrayFromIDs(buffer))
				.withAverageDistance(Meters.of(limelightEstimate.avgTagDist));
		LogUtil.log("Vision ESTIMATE", estimate.getPose());
		return Optional.of(List.of(estimate));
	}

	@Override
	public void logLocalizationSkipped(String reason) {
		diagnostics.observe(LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name), Drive.mInstance.getPose(),
				Timer.getFPGATimestamp(), DriverStation.isDisabled());
		diagnostics.decision(Decision.valueOf(reason), Drive.mInstance.getPose(), new double[0]);
	}

	@Override
	public void logVisionSubmitted(VisionEstimate estimate, Vector<N3> deviations, Pose2d poseAfter) {
		diagnostics.decision(Decision.FUSION_SUBMITTED, poseAfter,
				new double[] {deviations.get(0), deviations.get(1), deviations.get(2)});
	}

	@Override
	public void update() {
		outputTelemetry();
		updateGyro();
	}

	public void outputTelemetry() {
		lastGivenEstimate.ifPresent((lastEstimate) -> lastEstimate.log(config.name + "/Last Estimate"));
	}
}
