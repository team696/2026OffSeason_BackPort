package frc.lib.io.vision.limelight;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.LimelightHelpers;
import frc.lib.util.vision.LimelightHelpers.PoseEstimate;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTagLimelightCameraIO extends LimelightCameraIO {

	private static final double MAX_TAG_DISTANCE_METERS = 2.5;
	private static final double MAX_TAG_AMBIGUITY = 0.43;

	private Optional<VisionEstimate> lastGivenEstimate = Optional.empty();

	public AprilTagLimelightCameraIO(CameraIOConfig config) {
		super(config);
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

		  if (limelightEstimate != null && limelightEstimate.rawFiducials != null
				&& limelightEstimate.rawFiducials.length > 0 && limelightEstimate.rawFiducials[0] != null) {
        SmartDashboard.putNumber("Debug/Ambiguity", limelightEstimate.rawFiducials[0].ambiguity);
    		}
    	SmartDashboard.putNumber("Debug/TagCount", limelightEstimate == null ? -1 : limelightEstimate.tagCount);
		if (!isValidEstimate(limelightEstimate)) {
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
	public void update() {
		outputTelemetry();
		updateGyro();
	}

	public void outputTelemetry() {
		lastGivenEstimate.ifPresent((lastEstimate) -> lastEstimate.log(config.name + "/Last Estimate"));
	}
}
