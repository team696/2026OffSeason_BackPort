package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.subsystems.drive.Swerve;

import edu.wpi.first.wpilibj.DriverStation;


public class Binds {


	public static final class Controller {
			private static final SwerveRequest.FieldCentric swerveFCDriveRequest = new SwerveRequest.FieldCentric()
				.withDeadband(5. * 0.05).withRotationalDeadband(8 * 0.05)
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

		public static final void bind() {
			Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
				(()->swerveFCDriveRequest
				.withVelocityX(Controls.xboxController.leftJoyY.getAsDouble())
				.withVelocityY(Controls.xboxController.leftJoyX.getAsDouble())
				.withRotationalRate(Controls.xboxController.rightJoyX.getAsDouble()))
			));
		}
	}
}
