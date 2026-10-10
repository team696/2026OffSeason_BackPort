// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.LinkedHashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.FieldLayout;
import frc.robot.Auto.NamedCommand;
import frc.robot.Log.SEVERITY;
import frc.robot.commands.ShootingCommands;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.ShootingState;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperConstants.HopperPivot;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.SuperStructure;
import frc.robot.subsystems.Superstructure.SuperStructureConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraConstants.PipelineConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	public Robot() {

		Swerve.get();
		Shooter.get();
		Intake.RackAndPinion.get();
		Intake.Rollers.get();
		Intake.Rollers2.get();
		FeedRollers.get();
		Hood.get();
		Hopper.Pivot.get();
		Hopper.Rollers.get();
		//Log.initialize(SEVERITY.EVERYTHING);
		//Log.data("Swerve", Swerve.get(), SEVERITY.MINIMAL);

		Auto.configureAutoBuilder();
		Auto.initialize(
			new Auto.NamedCommand("Shoot", SuperStructure.Shooting()),

			new Auto.NamedCommand("Stop Shooting", SuperStructure.ShootingStop()),

			new Auto.NamedCommand("Intake", SuperStructure.RunIntake()),

			new Auto.NamedCommand("Stop Intake", SuperStructure.StopIntake())
		);

		Camera.get().setPipeline(PipelineConstants.APRIL_TAG);
		Binds.Controller.bind();
	}

	@Override
	public void robotInit(){
		
		// ---------- Intake ----------
		SmartDashboard.putData("Intake/Rack and Pinion/Extend", Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.EXTEND));
		SmartDashboard.putData("Intake/Rack and Pinion/Half", Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.HALF));
		SmartDashboard.putData("Intake/Rack and Pinion/Retract", Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.RETRACT));
		SmartDashboard.putData("Intake/Rollers/Intake", SuperStructureConstants.IntakeRollers());
		SmartDashboard.putData("Intake/Rollers/Outtake", SuperStructureConstants.OutakeRollers());
		SmartDashboard.putData("Intake/Rollers/Stop", SuperStructureConstants.StopIntakeRollers());

		// ---------- Shooter ----------
		SmartDashboard.putData("Shooter/Test Shot", Shooter.get().getTestShot());
		SmartDashboard.putData("Shooter/Shot", Shooter.get().setpointCommand(Shooter.SHOT));
		SmartDashboard.putData("Shooter/Ferry", Shooter.get().setpointCommand(Shooter.FERRY));
		SmartDashboard.putData("Shooter/Stop", Shooter.get().setpointCommand(Shooter.IDLE));

		// ---------- Superstructure ----------
		// shoot/Pass/AutoAlgin pick their ShootingState when they're created, not when clicked,
		// so withState() sets the right state at the moment the button is pressed
		SmartDashboard.putData("Superstructure/Run Intake", SuperStructure.RunIntake());
		SmartDashboard.putData("Superstructure/Outtake", SuperStructure.Outtake());
		SmartDashboard.putData("Superstructure/Stop Intake", SuperStructure.StopIntake());
		SmartDashboard.putData("Superstructure/Intake Rollers", SuperStructureConstants.IntakeRollers());
		SmartDashboard.putData("Superstructure/Outtake Rollers", SuperStructureConstants.OutakeRollers());
		SmartDashboard.putData("Superstructure/Stop Intake Rollers", SuperStructureConstants.StopIntakeRollers());
		SmartDashboard.putData("Superstructure/Oscillate Intake", SuperStructureConstants.OscilateIntake());
		SmartDashboard.putData("Superstructure/Run Hopper Roller", SuperStructureConstants.runHopperRoller());
		SmartDashboard.putData("Superstructure/Stop Hopper Roller", SuperStructureConstants.StopRoller());
		SmartDashboard.putData("Superstructure/Auto Align", withState(ShootingState.HUB, SuperStructure.AutoAlgin()));
		SmartDashboard.putData("Superstructure/Shooting", withState(ShootingState.HUB, SuperStructure.Shooting()));
		SmartDashboard.putData("Superstructure/Shooting Testing", SuperStructure.ShootingTesting());
		SmartDashboard.putData("Superstructure/Passing Left", withState(ShootingState.NEUTRAL_FERRY_LEFT, SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_LEFT)));
		SmartDashboard.putData("Superstructure/Passing Right", withState(ShootingState.NEUTRAL_FERRY_RIGHT, SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_RIGHT)));
		SmartDashboard.putData("Superstructure/Shooting Stop", SuperStructure.ShootingStop());
		/* 
		// ---------- Hopper ----------
		SmartDashboard.putData("Hopper/Rollers/Intake", Hopper.Rollers.get().setpointCommand(Hopper.Rollers.INTAKE));
		SmartDashboard.putData("Hopper/Rollers/Slow Intake", Hopper.Rollers.get().setpointCommand(Hopper.Rollers.SLOW_INTAKE));
		SmartDashboard.putData("Hopper/Rollers/Outtake", Hopper.Rollers.get().setpointCommand(Hopper.Rollers.OUTAKE));
		SmartDashboard.putData("Hopper/Rollers/Stop", Hopper.Rollers.get().setpointCommand(Hopper.Rollers.IDLE));

		SmartDashboard.putData("Hopper/Pivot/Up", Hopper.Pivot.get().setpointCommand(Hopper.Pivot.UP));
		SmartDashboard.putData("Hopper/Pivot/Down", Hopper.Pivot.get().setpointCommand(Hopper.Pivot.DOWN));
		SmartDashboard.putData("Hopper/Pivot/Test Angle", Hopper.Pivot.get().getTestAngleCommand());
		SmartDashboard.putData("Hopper/Pivot/Stop", Hopper.Pivot.get().setpointCommand(Hopper.Pivot.STOP));

		// ---------- Feed Rollers ----------
		SmartDashboard.putData("Feed Rollers/Run", FeedRollers.get().setpointCommand(FeedRollers.RUNNING));
		SmartDashboard.putData("Feed Rollers/Test Speed", FeedRollers.get().getTestSpeedCommand());
		SmartDashboard.putData("Feed Rollers/Stop", FeedRollers.get().setpointCommand(FeedRollers.IDLE));



		// ---------- Hood ----------
		SmartDashboard.putData("Hood/Test Angle", Hood.get().getTestShotCommand());
		SmartDashboard.putData("Hood/Max Angle", Hood.get().setpointCommand(Hood.MAX_ANGLE));
		SmartDashboard.putData("Hood/Stow", Hood.get().setpointCommand(Hood.STOW));

		// ---------- Vision ----------
		SmartDashboard.putData("Vision/AprilTag Pipeline", Camera.get().switchToAprilTagPipeline());
		SmartDashboard.putData("Vision/Neural Network Pipeline", Camera.get().switchToNueralNetworkPipeline());

		

		// ---------- Shooting Commands ----------
		SmartDashboard.putData("Shooting Commands/Spin Up", withState(ShootingState.HUB, ShootingCommands.spinUp()));
		SmartDashboard.putData("Shooting Commands/Test Spin Up", ShootingCommands.testspinUp());
		SmartDashboard.putData("Shooting Commands/Test Shot", ShootingCommands.testShot());
		SmartDashboard.putData("Shooting Commands/Shoot", withState(ShootingState.HUB, ShootingCommands.shoot()));
		SmartDashboard.putData("Shooting Commands/Pass Left", withState(ShootingState.NEUTRAL_FERRY_LEFT, ShootingCommands.Pass(ShootingState.NEUTRAL_FERRY_LEFT)));
		SmartDashboard.putData("Shooting Commands/Pass Right", withState(ShootingState.NEUTRAL_FERRY_RIGHT, ShootingCommands.Pass(ShootingState.NEUTRAL_FERRY_RIGHT)));
		SmartDashboard.putData("Shooting Commands/Stop", ShootingCommands.Stop());
		*/
	}

	// Sets the ShootingState when the button is clicked, then runs the command
	private static Command withState(ShootingState state, Command command) {
		return Commands.runOnce(() -> Shooting.setState(state)).andThen(command);
	}

	// Read-only handles for every motor, used to show each one's data on SmartDashboard (leaders and followers).
	// Each motor's data goes in the same folder as that subsystem's buttons.
	private final Map<String, TalonFX> dashboardMotors = new LinkedHashMap<>();
	{
		addDashboardMotor("Intake/Rollers/Leader", Ports.INTAKE_ROLLERS_MAIN);
		addDashboardMotor("Intake/Rollers/Follower", Ports.INTAKE_ROLLERS_FOLLOWER);
		addDashboardMotor("Intake/Rack and Pinion/Data", Ports.INTAKE_RACK_AND_PINION);
		addDashboardMotor("Hopper/Rollers/Data", Ports.HOPPER_ROLLERS);
		addDashboardMotor("Feed Rollers/Leader", Ports.FEEDER_ROLLERS);
		addDashboardMotor("Feed Rollers/Follower", Ports.FEED_ROLLERS_FOLLOWER);
		addDashboardMotor("Shooter/Leader", Ports.SHOOTER_MAIN);
		addDashboardMotor("Shooter/Follower", Ports.SHOOTER_FOLLOWER);
		addDashboardMotor("Hood/Data", Ports.HOOD);
		addDashboardMotor("Hopper/Pivot/Data", Ports.HOPPER_PIVOT);
	}

	private void addDashboardMotor(String folder, Ports port) {
		dashboardMotors.put(folder + " (" + port.id + ")", new TalonFX(port.id, port.bus));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		for (Map.Entry<String, TalonFX> entry : dashboardMotors.entrySet()) {
			String folder = entry.getKey() + "/";
			TalonFX motor = entry.getValue();
			SmartDashboard.putBoolean(folder + "Connected", motor.isConnected());
			SmartDashboard.putNumber(folder + "Velocity (rps)", motor.getVelocity().getValueAsDouble());
			SmartDashboard.putNumber(folder + "Position (rot)", motor.getPosition().getValueAsDouble());
			SmartDashboard.putNumber(folder + "Stator Current (A)", motor.getStatorCurrent().getValueAsDouble());
			SmartDashboard.putNumber(folder + "Supply Current (A)", motor.getSupplyCurrent().getValueAsDouble());
			SmartDashboard.putNumber(folder + "Voltage (V)", motor.getMotorVoltage().getValueAsDouble());
			SmartDashboard.putNumber(folder + "Temperature (C)", motor.getDeviceTemp().getValueAsDouble());
		}
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
