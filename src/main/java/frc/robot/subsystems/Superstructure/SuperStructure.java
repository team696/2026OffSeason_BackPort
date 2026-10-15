// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.commands.ShootingCommands;
import frc.robot.shooting.ShootingState;

/** Add your docs here. */
public class SuperStructure {
    public static Command RunIntake() {
        return Commands.parallel(
            Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.EXTEND),
            SuperStructureConstants.IntakeRollers()
        ).alongWith(Hopper.Rollers.get().setpointCommand(Hopper.Rollers.SLOW_INTAKE));
    }

    public static Command Outtake() {
        return Commands.parallel(
            Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.EXTEND),
            SuperStructureConstants.OutakeRollers()
        );
    }

    public static Command StopIntake(){
        return Commands.parallel(
            Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.RETRACT),
            SuperStructureConstants.StopIntakeRollers()
        );
    }
    
    // public static Command upwardExtendingHopper() {
    //     return Hopper.Upwards.get().setpointCommand(Hopper.Upwards.MOVE_HIGH);
    // }

    // public static Command lowerExtendingHopper() {
    //     return Hopper.Upwards.get().setpointCommand(Hopper.Upwards.MOVE_LOW);
    // }

    public static Command AutoAlgin(){
        return ShootingCommands.AutoAlgin();
    }

    public static Command Shooting(){
        return Commands.parallel(
            ShootingCommands.shoot(),
            Commands.waitSeconds(0.5),
                Commands.parallel(
                    Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.HALF),
                    SuperStructureConstants.SlowIntakeRollers(),
                    SuperStructureConstants.runHopperRoller()
                )
            );
    }

    public static Command ShootingTesting(){
        return Commands.parallel(
            ShootingCommands.testShot(),
            // Give the shooter 0.5s to spin up before the hopper starts feeding
            SuperStructureConstants.runHopperRoller().beforeStarting(Commands.waitSeconds(0.5))
        );
    }

    public static Command ShootingStop(){
        return Commands.parallel(
            ShootingCommands.Stop(),
            SuperStructureConstants.StopRoller()
        );
    }

    public static Command Passing(ShootingState side){
        return Commands.parallel(
            ShootingCommands.Pass(side),
            SuperStructureConstants.runHopperRoller()
        );
    }
}
