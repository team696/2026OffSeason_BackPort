//Original Swerve Code from 696 Templete
package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.drive.TunerConstants.TunerSwerveDrivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Swerve extends TunerSwerveDrivetrain implements Sendable, Subsystem{
    private static Swerve m_Swerve;

    private final SwerveSubsystem m_subsystemWrapper = new SwerveSubsystem();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    public Field2d fieldSim = new Field2d();

    public static synchronized Swerve get() {
        if (m_Swerve == null)
            m_Swerve = TunerConstants.createDrivetrain();
        return m_Swerve;
    }


    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, 0, modules);
        //Checks for simulation
        if (Utils.isSimulation()) {
            simulationInit();
        }
    }

    /* 
    Due to the changes in the new 2027 code base, SubsystemBase has become the new standard Class for subsystems. 
    Java's inheritance rules state you can only inherite ONE class, and that class being TunerSwerveDriveTrain,
    this class cannot support standard Command base methods such as run(), but most importantly setDefaultCommand().
    Hence, this method and the class metioned further down the file exists
    */
    public SubsystemBase toSubsystem() {
        return m_subsystemWrapper;
    }

    // This handles our standard periodic updates
    public void periodicUpdate() {
        //Red or Blue alliance checking base off if the robot is Disabled
        //This would be good for comps, since once FMS is connected the robot would be disabled
        //instead of defaulting to the Blue alliances no matter what alliance your on
        //if done in RobotInit(major problem last year Glendale)
        if (!m_hasAppliedOperatorPerspective || RobotState.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

    
        fieldSim.setRobotPose(this.getState().Pose);
        SmartDashboard.putData("field", fieldSim);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Value", () -> 1, null);
    }

    /**
     * Clean inline Command creation using our actual SubsystemBase requirement!
     */
    //Simple apply request
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return m_subsystemWrapper.run(() -> this.setControl(requestSupplier.get()));
    }

    //Uhhhhh i guess no stdDev?
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    //Supports the Time class instead of a double
    public void addVisionUpdate(Pose2d pose,Time timestamp,Matrix<N3,N1>stdDev){
        addVisionMeasurement(pose, timestamp.in(Seconds),stdDev);
    }

    //Pitch
    public boolean pitchStable() {
        return Math.abs(getPigeon2().getPitch().getValueAsDouble()) < 3.0;
        }

    //Simple calculations base off of its current position plus the speed times the current timestamp
    public Pose2d getLookAheadPoseWithoutHeading(Time dt) {
        Pose2d current = getState().Pose;
        var speeds = getState().Speeds;
        double t = dt.in(Seconds);
        return new Pose2d(
            current.getX() + speeds.vxMetersPerSecond * t,
            current.getY() + speeds.vyMetersPerSecond * t,
            current.getRotation()
    );
}

    double m_lastSimTime;
    Notifier simUpdate;

    private void simulationInit() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        simUpdate = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simUpdate.setName("Swerve Simulation Update");
        simUpdate.startPeriodic(0.005);
    }

    // 3. This nested inner class extends SubsystemBase cleanly and handles the scheduler loops
    private class SwerveSubsystem extends SubsystemBase {
        @Override
        public void periodic() {
            periodicUpdate();
        }
    }
}