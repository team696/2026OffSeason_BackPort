// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;

/** Add your docs here. */
public class SuperStructureConstants {

    public static Command runHopperRoller(){
        return Hopper.Rollers.get().setpointCommand(Hopper.Rollers.INTAKE);
    }

    public static Command StopRoller(){
        return Hopper.Rollers.get().setpointCommand(Hopper.Rollers.IDLE);
    }

    public static Command OscilateIntake(){
        return Commands.sequence(
            Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.EXTEND),
            Commands.waitSeconds(0.25),
            Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.HALF)
        );
    }

    public static Command IntakeRollers(){
        return Commands.parallel(
            Intake.Rollers.get().setpointCommand(Intake.Rollers.INTAKE),
            Intake.Rollers2.get().setpointCommand(Intake.Rollers2.INTAKE)
        );
    }

    public static Command SlowIntakeRollers(){
        return Commands.parallel(
            Intake.Rollers.get().setpointCommand(Intake.Rollers.SLOW_INTAKE),
            Intake.Rollers2.get().setpointCommand(Intake.Rollers2.SLOW_INTAKE)
        );
    }

    public static Command OutakeRollers(){
        return Commands.parallel(
            Intake.Rollers.get().setpointCommand(Intake.Rollers.OUTAKE),
            Intake.Rollers2.get().setpointCommand(Intake.Rollers2.OUTAKE)
        );
    }

    public static Command StopIntakeRollers(){
        return Commands.parallel(
            Intake.Rollers.get().setpointCommand(Intake.Rollers.IDLE),
            Intake.Rollers2.get().setpointCommand(Intake.Rollers2.IDLE)
        );
    }





}
