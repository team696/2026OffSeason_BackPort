// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Log.SEVERITY;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraConstants.PipelineConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	public Robot() {

		Swerve.get();
		Camera.get();

		//Log.initialize(SEVERITY.EVERYTHING);
		//Log.data("Swerve", Swerve.get(), SEVERITY.MINIMAL);

		// Auto.initialize(
		// 		new NamedCommand("PRINT", new InstantCommand(() -> System.out.println(1))));

		Camera.get().setPipeline(PipelineConstants.APRIL_TAG);
		Binds.Controller.bind();
	}

	@Override
	public void robotInit(){
		Shooter.get(); //temp
		Intake.RackAndPinion.get(); //also temp
		Intake.Rollers.get();
		Hopper.Rollers.get();
		Hopper.Upwards.get();
		FeedRollers.get();
		Hood.get();

	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// m_autonomousCommand = Auto.getSelectedAuto();

		// if (m_autonomousCommand != null) {
		// 	m_autonomousCommand.schedule();
		// }
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}


	
}
