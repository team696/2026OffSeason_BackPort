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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.sim.field.rebuilt.FieldSim;
import frc.lib.util.FieldLayout;
import frc.robot.Controls;
import frc.robot.RobotConstants;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.regression.AirtimeRegression;
import frc.robot.subsystems.drive.Swerve;

/** Add your docs here. */
public class MotionComp {
    private final Transform2d m_shooterOffset = RobotConstants.SHOOTER_OFFSET;
    private Pose2d m_currLookAhead = new Pose2d();

    //TODO: need to update relase time
    public static final Time SHOOTER_DELAY = Seconds.of(0.125);
    public static final AngularVelocity YAW_INSTABILITY = DegreesPerSecond.of(180);
    public static final Angle ERROR_TOLERANCE = Degrees.of(0.1);
    public static final double STATIC_SHOT_THRESHOLD = 0.09;

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

    private Distance getRobotDistance(){
        return Meters.of(getCurrentPose().getTranslation().getDistance(getTarget()));
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

    public ChassisSpeeds FindDirectionTangentialVelocity() {
        double vel_x = getCurrentSpeeds().vxMetersPerSecond
            + (m_shooterOffset.getX() * getCurrentSpeeds().omegaRadiansPerSecond * phi().getCos());
        double vel_y = getCurrentSpeeds().vyMetersPerSecond
            + (m_shooterOffset.getX() * getCurrentSpeeds().omegaRadiansPerSecond * phi().getSin());
        ChassisSpeeds Shooter_Velocity = new ChassisSpeeds(vel_x, vel_y,0.0);
        
        return Shooter_Velocity;
    }

    public Translation2d updatedPose(Distance distance){
        double x_pose = ProgressLookAheadPos().getX() + FindDirectionTangentialVelocity().vxMetersPerSecond * AirtimeRegression.distanceToTimeToHub.get(distance.in(Meters));
        double y_pose = ProgressLookAheadPos().getY() + FindDirectionTangentialVelocity().vyMetersPerSecond * AirtimeRegression.distanceToTimeToHub.get(distance.in(Meters));
        Translation2d newPose = new Translation2d(x_pose, y_pose);

        return newPose;
    }


    public Angle getDriveError(Translation2d hubTarget) {
        Rotation2d currentAngle = getCurrentPose().getRotation();
        Translation2d lookAhead = updatedPose(getRobotDistance());
        Rotation2d targetAngle = hubTarget.minus(lookAhead).getAngle(); 
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
            return updatedPose(getRobotDistance());
        }

        
    }

    public Rotation2d atan2(Translation2d pose){
       return new Rotation2d(Math.atan2(pose.getY(), pose.getX()));
    }
    
    public Command PointToTarget(){
        SwerveRequest.FieldCentricFacingAngle FCFARequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(RobotConstants.DriveConstants.MaxSpeed *0.05)
        .withRotationalDeadband(RobotConstants.DriveConstants.MaxSpeed*0.05)
        .withHeadingPID(3, 0, 0);
 
       return Commands.run(()->{ 
        Swerve.get().fieldSim.getObject("Target Pose").setPose(getTarget().getX(),getTarget().getY(),  getTarget().getAngle()); 

        Swerve.get().setControl(
            FCFARequest
            .withVelocityX(Controls.xboxController.leftJoyY.getAsDouble()/2*RobotConstants.DriveConstants.MaxSpeed)
            .withVelocityY(Controls.xboxController.leftJoyX.getAsDouble()/2*RobotConstants.DriveConstants.MaxSpeed)
            .withTargetDirection(atan2(getTarget().minus(getBaseTranslation()))));});
    }


}

