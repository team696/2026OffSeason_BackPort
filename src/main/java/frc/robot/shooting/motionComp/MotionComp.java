// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.motionComp;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.reflect.Field;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.sim.field.rebuilt.Hub;
import frc.lib.util.FieldLayout;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.shooting.regression.AirtimeRegression;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.TunerConstants;

/** Add your docs here. */
public class MotionComp {
    private final Transform2d m_shooterOffset = RobotConstants.SHOOTER_OFFSET;
    private Pose2d m_currLookAhead = new Pose2d();

    //TODO: need to update relase time
    public static final Time SHOOTER_DELAY = Seconds.of(0.0);


    MotionComp() {}

    private Pose2d getCurrentPose() {
        return Swerve.get().getState().Pose;
    }

    private ChassisSpeeds getCurrentSpeeds() {
        return Swerve.get().getState().Speeds;
    }

    private Distance getRobotDistance(){
        Translation2d tranlsation = getCurrentPose().getTranslation();
        double distanceRaw;
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
        {
            distanceRaw = tranlsation.getDistance(FieldLayout.kBlueHub);
        }
        else{
            distanceRaw = tranlsation.getDistance(FieldLayout.handleAllianceFlip(FieldLayout.kBlueHub,true));
        }
        Distance distanceInMeters = Meters.of(distanceRaw);
        return distanceInMeters;
    }

    
 
    Pose2d ProgressLookAheadPos() {
        Twist2d twist2d = new Twist2d(
            getCurrentSpeeds().vxMetersPerSecond * SHOOTER_DELAY.in(Seconds),
            getCurrentSpeeds().vyMetersPerSecond * SHOOTER_DELAY.in(Seconds),
            getCurrentSpeeds().omegaRadiansPerSecond * SHOOTER_DELAY.in(Seconds)
        );
        Pose2d new_pos = getCurrentPose().exp(twist2d);
        return new_pos;
    }

    Rotation2d phi() {
        Rotation2d theta = ProgressLookAheadPos().getRotation();
        Rotation2d phi = theta.plus(Rotation2d.fromRadians(Math.copySign(Math.PI/2.0, getCurrentSpeeds().omegaRadiansPerSecond)));

        return phi;
    }

    ChassisSpeeds FindDirectionTangentialVelocity() {
        double vel_x = getCurrentSpeeds().vxMetersPerSecond + m_shooterOffset.getX() +(SHOOTER_DELAY.in(Seconds) * getCurrentSpeeds().omegaRadiansPerSecond * phi().getCos());
        double vel_y = getCurrentSpeeds().vyMetersPerSecond + m_shooterOffset.getY() +(SHOOTER_DELAY.in(Seconds) * getCurrentSpeeds().omegaRadiansPerSecond * phi().getSin());
        ChassisSpeeds Shooter_Velocity = new ChassisSpeeds(vel_x, vel_y,0.0);
        
        return Shooter_Velocity;
    }

    Translation2d updatedPose(Distance distance){
        double x_pose = ProgressLookAheadPos().getX() + FindDirectionTangentialVelocity().vxMetersPerSecond * AirtimeRegression.distanceToTimeToHub.get(distance.in(Meters));
        double y_pose = ProgressLookAheadPos().getY() + FindDirectionTangentialVelocity().vyMetersPerSecond * AirtimeRegression.distanceToTimeToHub.get(distance.in(Meters));
        Translation2d newPose = new Translation2d(x_pose, y_pose);

        return newPose;
    }

    void rejectPose(){
        
    }

    Rotation2d atan2(Translation2d pose){
       return new Rotation2d(Math.atan2(pose.getX(), pose.getY()));
    }
    
    void PointToTarget(){
        SwerveRequest.FieldCentricFacingAngle FCFARequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(RobotConstants.DriveConstants.MaxSpeed)
        .withRotationalDeadband(RobotConstants.DriveConstants.MaxSpeed)
        .withHeadingPID(8, 0, 0)
        ;

        Swerve.get().setControl(
            FCFARequest
            .withVelocityX(Controls.xboxController.leftJoyX.getAsDouble()/2*RobotConstants.DriveConstants.MaxSpeed)
            .withVelocityY(Controls.xboxController.leftJoyY.getAsDouble()/2*RobotConstants.DriveConstants.MaxSpeed)
            .withTargetDirection(atan2(updatedPose(getRobotDistance())))
            );
    }


}

