package frc.lib.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import frc.lib.util.FieldLayout;
import frc.lib.util.vision.LimelightHelpers.PoseEstimate;
import frc.lib.util.vision.LimelightHelpers.RawFiducial;
import java.util.Arrays;

/** Observes vision decisions without adding rejection rules or changing estimator inputs. */
public final class VisionTelemetry {
    public enum Decision {
        FUSION_SUBMITTED, DISABLED_POSE_RESET, NULL_ESTIMATE, NULL_POSE,
        NO_TAGS, ORIGIN_POSE, HIGH_YAW_RATE, PITCH_UNSTABLE, VISION_DISABLED,
        INVALID_FRAME_TIME, DUPLICATE_OR_OUT_OF_ORDER, INVALID_ESTIMATE
    }

    private final NetworkTable table;
    private final StructPublisher<Pose2d> rawPose;
    private final StructPublisher<Pose2d> acceptedPose;
    private final StructPublisher<Pose2d> rejectedPose;
    private final StructPublisher<Pose2d> referencePose;
    private final StructPublisher<Pose2d> poseAfterSubmission;
    private final DoubleArrayPublisher events;
    private PoseEstimate frame;
    private Pose2d reference = new Pose2d();
    private double now;
    private double previousTimestamp = Double.NaN;
    private boolean duplicate;
    private long reads, uniqueFrames, duplicateReads, accepted, rejected, decisions;

    public VisionTelemetry(NetworkTable table) {
        this.table = table;
        rawPose = table.getStructTopic("RawPose", Pose2d.struct).publish();
        acceptedPose = table.getStructTopic("AcceptedPose", Pose2d.struct).publish();
        rejectedPose = table.getStructTopic("RejectedPose", Pose2d.struct).publish();
        referencePose = table.getStructTopic("ReferencePoseAtRead", Pose2d.struct).publish();
        poseAfterSubmission = table.getStructTopic("PoseAfterSubmission", Pose2d.struct).publish();
        events = table.getDoubleArrayTopic("DecisionEvent").publish(
            PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true), PubSubOption.periodic(0.02));
        table.getEntry("DecisionCodes").setStringArray(
            Arrays.stream(Decision.values()).map(Enum::name).toArray(String[]::new));
        table.getEntry("DecisionEventColumns").setStringArray(new String[] {
            "sequence", "decisionCode", "decisionFpgaSeconds", "captureFpgaSeconds", "ageSeconds",
            "rawX_m", "rawY_m", "rawHeading_rad", "referenceX_m", "referenceY_m", "referenceHeading_rad",
            "tagCount", "averageTagDistance_m", "maximumAmbiguity", "residualToCurrentPose_m",
            "headingResidualToCurrentPose_rad", "sigmaX_m", "sigmaY_m", "sigmaHeading_rad", "duplicateRead"
        });
        table.getEntry("AcceptedMeaning").setString("Submitted to fusion or used for reset; native estimator acceptance is not observable");
        table.getEntry("CountMeaning").setString("Processing attempts; inspect Decision for whether each read was submitted or rejected");
        table.getEntry("HasAcceptedPose").setBoolean(false);
        table.getEntry("HasRejectedPose").setBoolean(false);
        table.getEntry("AcceptedCount").setInteger(0);
        table.getEntry("RejectedCount").setInteger(0);
    }

    public void observe(PoseEstimate estimate, Pose2d currentPose, double timestamp, boolean disabled) {
        frame = estimate;
        reference = currentPose;
        now = timestamp;
        double capture = estimate == null ? Double.NaN : estimate.timestampSeconds;
        duplicate = estimate != null && Double.isFinite(capture)
            && Double.compare(capture, previousTimestamp) == 0;
        if (duplicate) duplicateReads++;
        else if (estimate != null && Double.isFinite(capture)) uniqueFrames++;
        boolean advanced = Double.isFinite(capture)
            && (!Double.isFinite(previousTimestamp) || capture > previousTimestamp);
        previousTimestamp = capture;
        table.getEntry("ReadCount").setInteger(++reads);
        table.getEntry("UniqueFrameCount").setInteger(uniqueFrames);
        table.getEntry("DuplicateReadCount").setInteger(duplicateReads);
        table.getEntry("DuplicateRead").setBoolean(duplicate);
        table.getEntry("TimestampAdvanced").setBoolean(advanced);
        table.getEntry("Disabled").setBoolean(disabled);
        table.getEntry("ReadFpgaSeconds").setDouble(now);
        table.getEntry("CaptureFpgaSeconds").setDouble(capture);
        table.getEntry("AgeSeconds").setDouble(now - capture);
        table.getEntry("StaleOver250msDiagnostic").setBoolean(Double.isFinite(capture) && now - capture > 0.25);
        table.getEntry("TimestampInFutureDiagnostic").setBoolean(Double.isFinite(capture) && capture - now > 0.05);
        table.getEntry("RawPoseAvailable").setBoolean(estimate != null && estimate.pose != null);
        referencePose.set(currentPose);
        boolean finite = estimate != null && finite(estimate.pose);
        table.getEntry("FinitePoseDiagnostic").setBoolean(finite);
        table.getEntry("InsideConfiguredFieldBoundsDiagnostic").setBoolean(finite && !FieldLayout.outsideField(estimate.pose));
        if (estimate != null && estimate.pose != null) rawPose.set(estimate.pose);
        table.getEntry("TagCount").setInteger(estimate == null ? 0 : estimate.tagCount);
        table.getEntry("LatencyMilliseconds").setDouble(estimate == null ? Double.NaN : estimate.latency);
        table.getEntry("AverageTagDistanceMeters").setDouble(estimate == null ? Double.NaN : estimate.avgTagDist);
        table.getEntry("TagSpanMeters").setDouble(estimate == null ? Double.NaN : estimate.tagSpan);
        table.getEntry("AverageTagArea").setDouble(estimate == null ? Double.NaN : estimate.avgTagArea);
        RawFiducial[] fiducials = estimate == null || estimate.rawFiducials == null
            ? new RawFiducial[0] : Arrays.stream(estimate.rawFiducials).filter(f -> f != null).toArray(RawFiducial[]::new);
        table.getEntry("RawFiducialsComplete").setBoolean(estimate != null && fiducials.length == estimate.tagCount);
        table.getEntry("TagIDs").setIntegerArray(Arrays.stream(fiducials).mapToLong(f -> f.id).toArray());
        table.getEntry("TagAmbiguities").setDoubleArray(Arrays.stream(fiducials).mapToDouble(f -> f.ambiguity).toArray());
        table.getEntry("TagDistancesToCameraMeters").setDoubleArray(Arrays.stream(fiducials).mapToDouble(f -> f.distToCamera).toArray());
        table.getEntry("MaximumAmbiguity").setDouble(maximumAmbiguity());
        table.getEntry("ResidualToCurrentPoseMeters").setDouble(residual());
        table.getEntry("HeadingResidualToCurrentPoseDegrees").setDouble(Math.toDegrees(headingResidual()));
        table.getEntry("Decision").setString("PENDING");
        table.getEntry("AcceptedThisRead").setBoolean(false);
        table.getEntry("SubmittedStdDevs").setDoubleArray(new double[0]);
    }

    public void decision(Decision decision, Pose2d poseAfter, double[] sigmas) {
        boolean submitted = decision == Decision.FUSION_SUBMITTED || decision == Decision.DISABLED_POSE_RESET;
        table.getEntry("Decision").setString(decision.name());
        table.getEntry("AcceptedThisRead").setBoolean(submitted);
        table.getEntry("DecisionSequence").setInteger(++decisions);
        table.getEntry("LastDecisionFpgaSeconds").setDouble(now);
        table.getEntry("SubmittedStdDevs").setDoubleArray(sigmas);
        table.getEntry("UsesFusionStdDevs").setBoolean(decision == Decision.FUSION_SUBMITTED);
        if (submitted) {
            table.getEntry("AcceptedCount").setInteger(++accepted);
            table.getEntry("LastAcceptedDecision").setString(decision.name());
            table.getEntry("LastAcceptedStdDevs").setDoubleArray(sigmas);
            table.getEntry("LastAcceptedCaptureFpgaSeconds").setDouble(frame == null ? Double.NaN : frame.timestampSeconds);
            table.getEntry("LastAcceptedDecisionFpgaSeconds").setDouble(now);
        } else {
            table.getEntry("RejectedCount").setInteger(++rejected);
            table.getEntry("LastRejectedReason").setString(decision.name());
            table.getEntry("LastRejectedCaptureFpgaSeconds").setDouble(frame == null ? Double.NaN : frame.timestampSeconds);
            table.getEntry("LastRejectedDecisionFpgaSeconds").setDouble(now);
        }
        boolean hasPose = frame != null && frame.pose != null;
        table.getEntry(submitted ? "LastAcceptedHadPose" : "LastRejectedHadPose").setBoolean(hasPose);
        if (hasPose) {
            (submitted ? acceptedPose : rejectedPose).set(frame.pose);
            table.getEntry(submitted ? "HasAcceptedPose" : "HasRejectedPose").setBoolean(true);
        }
        poseAfterSubmission.set(poseAfter);
        events.set(new double[] {
            decisions, decision.ordinal(), now, frame == null ? Double.NaN : frame.timestampSeconds,
            frame == null ? Double.NaN : now-frame.timestampSeconds,
            hasPose ? frame.pose.getX() : Double.NaN, hasPose ? frame.pose.getY() : Double.NaN,
            hasPose ? frame.pose.getRotation().getRadians() : Double.NaN,
            reference.getX(), reference.getY(), reference.getRotation().getRadians(),
            frame == null ? 0 : frame.tagCount, frame == null ? Double.NaN : frame.avgTagDist,
            maximumAmbiguity(), residual(), headingResidual(),
            sigmas.length == 3 ? sigmas[0] : Double.NaN, sigmas.length == 3 ? sigmas[1] : Double.NaN,
            sigmas.length == 3 ? sigmas[2] : Double.NaN, duplicate ? 1 : 0
        });
    }

    private double residual() {
        return frame == null || frame.pose == null ? Double.NaN
            : frame.pose.getTranslation().getDistance(reference.getTranslation());
    }

    private double headingResidual() {
        return frame == null || frame.pose == null ? Double.NaN
            : frame.pose.getRotation().minus(reference.getRotation()).getRadians();
    }

    private double maximumAmbiguity() {
        if (frame == null || frame.rawFiducials == null) return Double.NaN;
        return Arrays.stream(frame.rawFiducials).filter(f -> f != null).mapToDouble(f -> f.ambiguity).max().orElse(Double.NaN);
    }

    private static boolean finite(Pose2d pose) {
        return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians());
    }

    public void gyro(double yawDegrees, double pitchDegrees, double yawRateDegreesPerSecond,
                     double yawSampleAgeSeconds, double pitchSampleAgeSeconds) {
        table.getEntry("GyroYawDegrees").setDouble(yawDegrees);
        table.getEntry("GyroPitchDegrees").setDouble(pitchDegrees);
        table.getEntry("GyroYawRateDegreesPerSecond").setDouble(yawRateDegreesPerSecond);
        table.getEntry("GyroYawSampleAgeSeconds").setDouble(yawSampleAgeSeconds);
        table.getEntry("GyroPitchSampleAgeSeconds").setDouble(pitchSampleAgeSeconds);
    }
}
