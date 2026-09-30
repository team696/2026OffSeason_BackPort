// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.FieldLayout;
import frc.robot.Auto.NamedCommand;
import frc.robot.Log.SEVERITY;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.SuperStructure;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraConstants.PipelineConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	public Robot() {

		//Swerve.get();
		 Shooter.get(); //temp
		 Intake.RackAndPinion.get(); //also temp
		 //Intake.Rollers.get();
		 //Hopper.Rollers.get();
		 //Hopper.Upwards.get();
		 //FeedRollers.get();
		 //Hood.get();

		//Log.initialize(SEVERITY.EVERYTHING);
		//Log.data("Swerve", Swerve.get(), SEVERITY.MINIMAL);

		// Auto.configureAutoBuilder();
		// Auto.initialize(new Auto.NamedCommand("Funny auto", new PathPlannerAuto("Test Auto")));

		//Camera.get().setPipeline(PipelineConstants.APRIL_TAG);
		//Binds.Controller.bind();
	}

	@Override
	public void robotInit(){
		
		// SmartDashboard.putData("Testing autoalign", ShootingCommands.shoot());
		// SmartDashboard.putData("Testing Ferry", ShootingCommands.Pass());
		SmartDashboard.putData("Testing Intake", Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.EXTEND));
		SmartDashboard.putData("Retracting Intake", Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.RETRACT));
		SmartDashboard.putData("Testing Roller Intake", Intake.Rollers.get().setpointCommand(Intake.Rollers.INTAKE));
		SmartDashboard.putData("Testing Roller Outtake", Intake.Rollers.get().setpointCommand(Intake.Rollers.OUTAKE));
		//SmartDashboard.putData("Stop Intake Roller", Intake.Rollers.get().setpointCommand(Intake.Rollers.IDLE));
		SmartDashboard.putData("Stop intake", Intake.Rollers.get().setpointCommand(Intake.Rollers.IDLE));
		SmartDashboard.putData("Hood Test", Hood.get().getTestShotCommand());
		SmartDashboard.putData("Hood Zero", Hood.get().setpointCommand(Hood.STOW));
		SmartDashboard.putData("Shooter TestShot", Shooter.get().getTestShot());
		SmartDashboard.putData("Shooter Stop", Shooter.get().setpointCommand(Shooter.IDLE));

		SmartDashboard.putData("Test Spin up ", ShootingCommands.testspinUp());


		SmartDashboard.putData("Hopper Roller Intake", Hopper.Rollers.get().setpointCommand(Hopper.Rollers.INTAKE));
		SmartDashboard.putData("Hopper Roller Output", Hopper.Rollers.get().setpointCommand(Hopper.Rollers.OUTAKE));
		SmartDashboard.putData("Hopper Roller Slow Intake", Hopper.Rollers.get().setpointCommand(Hopper.Rollers.SLOW_INTAKE));
		SmartDashboard.putData("Hopper Roller Stop", Hopper.Rollers.get().setpointCommand(Hopper.Rollers.IDLE));
		// SmartDashboard.putData("Upward Extending Out", Hopper.Upwards.get().setpointCommand(Hopper.Upwards.MOVE_HIGH));
		// SmartDashboard.putData("Upward Extending In", Hopper.Upwards.get().setpointCommand(Hopper.Upwards.MOVE_LOW));
		SmartDashboard.putData("Test Feed Rollers", FeedRollers.get().setpointCommand(FeedRollers.RUNNING));
		SmartDashboard.putData("Feed Rollers stop", FeedRollers.get().setpointCommand(FeedRollers.IDLE));
		SmartDashboard.putNumber("Distance to Hub", Swerve.get().distTo(FieldLayout.kBlueHub));

		// SmartDashboard.putData("Testing Shooting Sequence", SuperStructure.Shooting());
		SmartDashboard.putData("Testing Auto Algin Sequence", SuperStructure.AutoAlgin());

		SmartDashboard.putData("Testing Intake SuperStructure", SuperStructure.RunIntake());
		SmartDashboard.putData("Testing Intake SuperStucure Idle", SuperStructure.StopIntake());
		SmartDashboard.putData("Testing Shooter SuperStructure", SuperStructure.ShootingTesting());
		SmartDashboard.putData("Testing Shooter Stoping", SuperStructure.ShootingStop());
	} 

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = Auto.getSelectedAuto();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}


	
}
