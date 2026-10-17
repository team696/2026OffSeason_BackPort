package frc.lib.io.vision.limelight;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.LimelightHelpers;
import frc.lib.util.vision.LimelightHelpers.PoseEstimate;
import frc.lib.util.vision.LimelightHelpers.RawDetection;
import frc.lib.util.vision.VisionEstimate;
import frc.lib.util.vision.VisionGamePiece;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Swerve;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class CombinedLimelightCameraIO extends LimelightCameraIO {

	// MegaTag2 estimates are unreliable while spinning fast
	private static final double MAX_MT2_YAW_RATE_DEG_PER_SEC = 720.0;

	private Optional<VisionEstimate> lastEstimate = Optional.empty();
	private final NetworkTable table;

	public CombinedLimelightCameraIO(CameraIOConfig config) {
		super(config);
		table = NetworkTableInstance.getDefault().getTable("Vision").getSubTable(config.name);
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

	@Override
	public Optional<List<VisionEstimate>> getLastEstimates() {
		// Disabled: MegaTag1 seeds the full pose (including heading) so MegaTag2 gets a correct field heading
		if (DriverStation.isDisabled()) {
			PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
			if (mt1 != null) {
				Swerve.get().resetPose(mt1.pose);
			}
			return Optional.empty();
		}

		// Enabled: MegaTag2 for x/y, heading stays on the Pigeon
		if (Math.abs(getYawRate()) > MAX_MT2_YAW_RATE_DEG_PER_SEC) {
			return Optional.empty();
		}
		PoseEstimate limelightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name); // _MegaTag2 TODO: Add a variable to properly fix this
		if (limelightEstimate == null
				|| (limelightEstimate.pose.getX() == 0
						|| limelightEstimate.pose.getY() == 0
						|| limelightEstimate.tagCount < 1)) {
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
		outputTelemetry();
	}

	public void outputTelemetry() {
		lastEstimate.ifPresent((lastEstimate) -> lastEstimate.log(config.name + "/Last Estimate"));
	}
}
