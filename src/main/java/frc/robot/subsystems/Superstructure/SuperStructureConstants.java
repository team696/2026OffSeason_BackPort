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
        return Commands.run(()->{Hopper.Rollers.get().setpointCommand(Hopper.Rollers.INTAKE);});
    }

    public static Command StopRoller(){
        return Commands.run(()->{Hopper.Rollers.get().setpointCommand(Hopper.Rollers.IDLE);});
    }

    public static Command OscilateIntake(){
        return Commands.sequence(
            Commands.run(()->{Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.EXTEND);}),
            Commands.waitSeconds(0.25),
            Commands.run(()->{Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.HALF);})
        );
    }



}
