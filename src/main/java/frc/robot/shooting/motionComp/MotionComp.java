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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Swerve;

/** Add your docs here. */
public class MotionComp {
    private final Transform2d m_shooterOffset = RobotConstants.SHOOTER_OFFSET;
    private Pose2d m_currLookAhead = new Pose2d();
    private Translation2d m_currentLookAhead = new Translation2d();

    //TODO: need to update relase time
    public static final Time SHOOTER_DELAY = Seconds.of(0.0);

    Pose2d pose = Swerve.get().getState().Pose;
    ChassisSpeeds velocities = new ChassisSpeeds();

    MotionComp() {}
 
    Pose2d ProgressLookAheadPos() {
        Twist2d twist2d = new Twist2d(
            velocities.vxMetersPerSecond * SHOOTER_DELAY.in(Seconds),
            velocities.vyMetersPerSecond * SHOOTER_DELAY.in(Seconds),
            velocities.omegaRadiansPerSecond * SHOOTER_DELAY.in(Seconds)
        );
        Pose2d new_pos = pose.exp(twist2d);
        return new_pos;
    }

    Rotation2d phi() {
        Rotation2d theta = ProgressLookAheadPos().getRotation();
        Rotation2d phi = theta.plus(Rotation2d.fromRadians(Math.copySign(Math.PI/2.0, velocities.omegaRadiansPerSecond)));

        return phi;
    }

    ChassisSpeeds FindDirectionTangentialVelocity() {
        double vel_x = velocities.vxMetersPerSecond + (SHOOTER_DELAY.in(Seconds) * velocities.omegaRadiansPerSecond * phi().getCos());
        double vel_y = velocities.vyMetersPerSecond + (SHOOTER_DELAY.in(Seconds) * velocities.omegaRadiansPerSecond * phi().getSin());
        ChassisSpeeds Shooter_Velocity = new ChassisSpeeds(vel_x, vel_y,0.0);
        
        return Shooter_Velocity;
    }


}

