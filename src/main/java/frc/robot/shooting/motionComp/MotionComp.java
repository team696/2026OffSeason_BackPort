// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.motionComp;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Controls;
import frc.robot.RobotConstants;
import frc.robot.shooting.Shooting;
import frc.robot.subsystems.drive.Swerve;

/** Add your docs here. */
public class MotionComp {
    private final Transform2d m_shooterOffset = RobotConstants.SHOOTER_OFFSET;
    //private Pose2d m_currLookAhead = new Pose2d();


    public static final Time SHOOTER_DELAY = Seconds.of(0.125);
    public static final AngularVelocity YAW_INSTABILITY = DegreesPerSecond.of(180);
    public static final Angle ERROR_TOLERANCE = Degrees.of(1);
    public static final double STATIC_SHOT_THRESHOLD = 0.1;
    // Shots were landing consistently right, so aim this much to the left (positive = counterclockwise = left)
    public static final Rotation2d AIM_OFFSET = Rotation2d.fromDegrees(3.0);

    private boolean m_rejectLookAhead = false;


    public MotionComp() {}

    private Pose2d getCurrentPose() {
        return Swerve.get().getState().Pose;
    }

    private ChassisSpeeds getCurrentSpeeds() {
        return Swerve.get().getState().Speeds;
    }

    public Translation2d getTarget() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return Shooting.getState().getLocationTranslation(isRed);
    }

    
 
    public Pose2d ProgressLookAheadPos() {
        Twist2d twist2d = new Twist2d(
            getCurrentSpeeds().vxMetersPerSecond * SHOOTER_DELAY.in(Seconds),
            getCurrentSpeeds().vyMetersPerSecond * SHOOTER_DELAY.in(Seconds),
            getCurrentSpeeds().omegaRadiansPerSecond * SHOOTER_DELAY.in(Seconds)
        );
        Pose2d new_pos = getCurrentPose().exp(twist2d);
        return new_pos;
    }

    public Rotation2d phi() {
        Rotation2d phi = ProgressLookAheadPos().getRotation();
        return phi.plus(Rotation2d.fromRadians(Math.PI / 2.0));

    }

    private ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getCurrentSpeeds(), getCurrentPose().getRotation());
    }

    public ChassisSpeeds FindDirectionTangentialVelocity() {
        ChassisSpeeds fieldSpeeds = getFieldSpeeds();
        double vel_x = fieldSpeeds.vxMetersPerSecond
            + (m_shooterOffset.getX() * fieldSpeeds.omegaRadiansPerSecond * phi().getCos());
        double vel_y = fieldSpeeds.vyMetersPerSecond
            + (m_shooterOffset.getX() * fieldSpeeds.omegaRadiansPerSecond * phi().getSin());
        ChassisSpeeds Shooter_Velocity = new ChassisSpeeds(vel_x, vel_y,0.0);

        return Shooter_Velocity;
    }

    public static final int AIRTIME_ITERATIONS = 5;

    // Airtime depends on the compensated distance, which depends on airtime, so iterate until they agree
    public Translation2d updatedPose(){
        Translation2d start = ProgressLookAheadPos().getTranslation();
        ChassisSpeeds shooterVelocity = FindDirectionTangentialVelocity();
        Translation2d target = getTarget();

        Translation2d newPose = start;
        for (int i = 0; i < AIRTIME_ITERATIONS; i++) {
            double airtime = Shooting.getState().getDistanceToFuelAirTimeMap().get(newPose.getDistance(target));
            newPose = new Translation2d(
                start.getX() + shooterVelocity.vxMetersPerSecond * airtime,
                start.getY() + shooterVelocity.vyMetersPerSecond * airtime);
        }

        return newPose;
    }


    public Angle getDriveError(Translation2d hubTarget) {
        Rotation2d currentAngle = getCurrentPose().getRotation();
        Translation2d lookAhead = updatedPose();
        Rotation2d targetAngle = hubTarget.minus(lookAhead).getAngle().plus(AIM_OFFSET);
        return targetAngle.minus(currentAngle).getMeasure(); 
    }

    public boolean errorTolerance(Angle error){
        return error.abs(Degrees) <= ERROR_TOLERANCE.in(Degrees);
    }

    public boolean getYawInstability(){
        return Math.abs(Swerve.get().getState().Speeds.omegaRadiansPerSecond) >= YAW_INSTABILITY.in(RadiansPerSecond);
    }

    public boolean staticShot(){
         return Math.hypot(Swerve.get().getState().Speeds.vxMetersPerSecond, Swerve.get().getState().Speeds.vyMetersPerSecond) < STATIC_SHOT_THRESHOLD;
    }

    public void rejectPose(){
        boolean yawUnstable = getYawInstability();
        boolean errorTolerance = errorTolerance(getDriveError(getTarget()));
        boolean staticShot = staticShot();
         
        m_rejectLookAhead = (yawUnstable && errorTolerance) || staticShot;
    }

    public boolean getRejectPose(){
        return m_rejectLookAhead;
    }

    public Translation2d getBaseTranslation() {
        rejectPose();

        if (m_rejectLookAhead) {
            return getCurrentPose().getTranslation();
        } else {
            return updatedPose();
        }

        
    }

    public Rotation2d atan2(Translation2d pose){
       return new Rotation2d(Math.atan2(pose.getY(), pose.getX()));
    }
    
    // How fast the aim angle is moving (rad/s) as the robot drives, so heading doesn't lag behind it
    public double getTargetRateFeedforward() {
        Translation2d r = getTarget().minus(getBaseTranslation());
        ChassisSpeeds fieldSpeeds = getFieldSpeeds();
        double distSq = r.getNorm() * r.getNorm();
        if (distSq < 1e-6) return 0.0;
        return (r.getY() * fieldSpeeds.vxMetersPerSecond - r.getX() * fieldSpeeds.vyMetersPerSecond) / distSq;
    }

    public Command PointToTarget(){
        SwerveRequest.FieldCentricFacingAngle FCFARequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(RobotConstants.DriveConstants.MaxSpeed *0.05)
        .withRotationalDeadband(0.02)
        .withHeadingPID(3, 0, 0)
        // Target angle is computed in field (blue) coordinates, so don't rotate it by the operator perspective (180 deg on red)
        .withTargetDirectionPerspective(SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance);
 
       return Commands.run(()->{ 
        Swerve.get().fieldSim.getObject("Target Pose").setPose(getTarget().getX(),getTarget().getY(),  getTarget().getAngle()); 

        Swerve.get().setControl(
            FCFARequest
            .withVelocityX(Controls.xboxController.leftJoyY.getAsDouble()/2*RobotConstants.DriveConstants.MaxSpeed)
            .withVelocityY(Controls.xboxController.leftJoyX.getAsDouble()/2*RobotConstants.DriveConstants.MaxSpeed)
            .withTargetDirection(atan2(getTarget().minus(getBaseTranslation())).plus(AIM_OFFSET))
            .withTargetRateFeedforward(getTargetRateFeedforward()));});
    }


}

