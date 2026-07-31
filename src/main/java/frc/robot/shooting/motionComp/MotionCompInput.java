// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.motionComp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.ChassisAccelerations;
import frc.robot.shooting.Shooting.ShootingState;

/** Add your docs here. */
public record MotionCompInput() {

    public void telemtrize(){
        SmartDashboard.putString(null, null);
    }

    //Will set up in a second
    // public static MotionCompInput zero() {
	// 	return new MotionCompInput(
	// 			ShootingState.HUB, new ChassisSpeeds(), Pose2d.kZero, new ChassisAccelerations(), 0.0);
	// }

}
