package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ZeroingCommands;
import frc.robot.shooting.ShootingState;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.SuperStructure;
import frc.robot.subsystems.Superstructure.SuperStructureConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.TunerConstants;



public class Binds {


	public static final class Controller {
			private static final SwerveRequest.FieldCentric swerveFCDriveRequest = new SwerveRequest.FieldCentric()
				.withDeadband(5. * 0.05).withRotationalDeadband(4 * 0.05)
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

		public static final void bind() {
			Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
				(()->swerveFCDriveRequest
				.withVelocityX(Math.pow(Controls.xboxController.leftJoyY.getAsDouble(),2)*Math.signum(Controls.xboxController.leftJoyY.getAsDouble())*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
				.withVelocityY(Math.pow(Controls.xboxController.leftJoyX.getAsDouble(),2)*Math.signum(Controls.xboxController.leftJoyX.getAsDouble())*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
				.withRotationalRate(Math.pow(Controls.xboxController.rightJoyX.getAsDouble(),2)*Math.signum(Controls.xboxController.rightJoyX.getAsDouble())*RotationsPerSecond.of(3.).in(RadiansPerSecond)))
			));

			//Intake.RackAndPinion.get().setDefaultCommand(Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.RETRACT));
			//Intake.Rollers.get().setDefaultCommand(Intake.Rollers.get().setpointCommand(Intake.Rollers.IDLE));
			//Intake.Rollers2.get().setDefaultCommand(Intake.Rollers2.get().setpointCommand(Intake.Rollers2.IDLE));
			Hood.get().setDefaultCommand(Hood.get().setpointCommand(Hood.STOW));
			Shooter.get().setDefaultCommand(Shooter.get().setpointCommand(Shooter.IDLE));
			Hopper.Rollers.get().setDefaultCommand(Hopper.Rollers.get().setpointCommand(Hopper.Rollers.IDLE));
			FeedRollers.get().setDefaultCommand(FeedRollers.get().setpointCommand(FeedRollers.IDLE));
			Controls.xboxController.X.whileTrue(ZeroingCommands.zeroIfNeeded());
			Controls.xboxController.LT.whileTrue(SuperStructure.RunIntake()).whileFalse(SuperStructureConstants.StopIntakeRollers());
			Controls.xboxController.RT.whileTrue(SuperStructure.Shooting());
			Controls.xboxController.Y.whileTrue(SuperStructure.ManualShooting());
			Controls.xboxController.A.whileTrue(Swerve.get().resetGyro());
			Controls.xboxController.LB.whileTrue(SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_LEFT));
			Controls.xboxController.RB.whileTrue(SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_RIGHT));
			Controls.xboxController.B.and(() -> !DriverStation.isTest())
				.whileTrue(Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.RETRACT));
			// Temporary Test-mode override: hold B to run only the two intake rollers.
			Controls.xboxController.B.and(DriverStation::isTestEnabled).whileTrue(
				Commands.startEnd(
					() -> {
						Intake.Rollers.get().applySetpoint(Intake.Rollers.INTAKE);
						Intake.Rollers2.get().applySetpoint(Intake.Rollers2.INTAKE);
					},
					() -> {
						Intake.Rollers.get().applySetpoint(Intake.Rollers.IDLE);
						Intake.Rollers2.get().applySetpoint(Intake.Rollers2.IDLE);
					},
					Intake.Rollers.get(), Intake.Rollers2.get()
				).withName("Test Intake Rollers Only")
			);
		}
	}
}
