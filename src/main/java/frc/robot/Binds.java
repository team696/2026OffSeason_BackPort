package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.shooting.ShootingState;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.SuperStructure;
import frc.robot.subsystems.drive.Swerve;



public class Binds {


	public static final class Controller {
			private static final SwerveRequest.FieldCentric swerveFCDriveRequest = new SwerveRequest.FieldCentric()
				.withDeadband(5. * 0.05).withRotationalDeadband(8 * 0.05)
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

		public static final void bind() {
			Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
				(()->swerveFCDriveRequest
				.withVelocityX(Controls.xboxController.leftJoyY.getAsDouble()*2)
				.withVelocityY(Controls.xboxController.leftJoyX.getAsDouble()*2)
				.withRotationalRate(Controls.xboxController.rightJoyX.getAsDouble()*2))
			));

			//Intake.RackAndPinion.get().setDefaultCommand(Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.RETRACT));
			// Hood.get().setDefaultCommand(Hood.get().setpointCommand(Hood.STOW));
			// Shooter.get().setDefaultCommand(Shooter.get().setpointCommand(Shooter.IDLE));

			Controls.xboxController.LT.whileTrue(SuperStructure.RunIntake());
			Controls.xboxController.RT.whileTrue(SuperStructure.Shooting());
			Controls.xboxController.A.whileTrue(SuperStructure.AutoAlgin());
			Controls.xboxController.X.whileTrue(SuperStructure.Outtake());
			Controls.xboxController.LT.whileTrue(SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_LEFT));
			Controls.xboxController.RT.whileTrue(SuperStructure.Passing(ShootingState.NEUTRAL_FERRY_RIGHT));
			
		}
	}
}
