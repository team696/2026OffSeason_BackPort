package frc.robot;


/*
    Due to the current situation with Choreo and Pathplanner not being supported on the new 2027 Alpha 5 code base
    and any version prior to Alpha 2 needing to be ported over anyways, disregard this file or any file related to 
    autos. We are currently either waiting for a new updated, or a port. 

*/

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.Namedwpilibj2.commands;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import frc.robot.subsystems.drive.Swerve;

// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStationErrors;
// // import edu.wpi.first.smartdashboard.SendableChooser;
// import edu.wpi.first.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// public class Auto {
// 	public static boolean initialized = false;

// 	public static SendableChooser<wpilibj2.command> autoChooser;

// 	public static void initialize(Namedwpilibj2.command... wpilibj2.commands) {
// 		if (initialized == true) {
// 			return;
// 		}

// 		for (Namedwpilibj2.command wpilibj2.command : wpilibj2.commands) 
// 		  wpilibj2.command.register();

// 		if (configureAutoBuilder()) {
// 			autoChooser = AutoBuilder.buildAutoChooser();
// 			SmartDashboard.putData("Auto Chooser", autoChooser);
// 		}

// 		initialized = true;
// 	}

// 	public static wpilibj2.command getSelectedAuto() {
// 		if (!initialized)
// 			return null;

// 		return autoChooser.getSelected();
// 	}

// 	public static final class Namedwpilibj2.command {
// 		public String name;
// 		public wpilibj2.command wpilibj2.command;

// 		/**
// 		 * @param name    to be used inside of the pathplanner GUI
// 		 * @param wpilibj2.command to be run when called inside pathplanner
// 		 */
// 		public Namedwpilibj2.command(String name, wpilibj2.command wpilibj2.command) {
// 			this.name = name;
// 			this.wpilibj2.command = wpilibj2.command;
// 		}

// 		public void register() {
// 			Namedwpilibj2.commands.registerwpilibj2.command(name, wpilibj2.command);
// 		}
// 	}

// 	private static boolean configureAutoBuilder() {
// 		try {
// 			var config = RobotConfig.fromGUISettings();
// 			AutoBuilder.configure(
// 					() -> Swerve.get().getState().Pose, // Supplier of current robot pose
// 					Swerve.get()::resetPose, // Consumer for seeding pose against auto
// 					() -> Swerve.get().getState().Velocity, // Supplier of current robot speeds
// 					// Consumer of ChassisSpeeds and feedforwards to drive the robot
// 					(speeds, feedforwards) -> Swerve.get().setControl(
// 							m_pathApplyRobotSpeeds.withSpeeds(speeds)
// 									.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
// 									.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
// 					new PPHolonomicDriveController(
// 							new PIDConstants(10, 0, 0),
// 							new PIDConstants(7, 0, 0)),
// 					config,
// 					// Assume the path needs to be flipped for Red vs Blue, this is normally the
// 					// case
// 					() -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
// 					Swerve.get() // Subsystem for requirements
// 			);
// 		} catch (Exception ex) {
// 			DriverStationErrors.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
// 			return false;
// 		}

// 		return true;
// 	}

// 	private static final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
// }
