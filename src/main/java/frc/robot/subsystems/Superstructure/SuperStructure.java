// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootingCommands;

/** Add your docs here. */
public class SuperStructure extends SubsystemBase{

    public Command AutoAlgin(){
        return ShootingCommands.AutoAlgin();
    }

    public Command Shooting(){
        return Commands.parallel(
            ShootingCommands.shoot(),
            SuperStructureConstants.runHopperRoller(),
            SuperStructureConstants.OscilateIntake()
        );
    }

    public Command ShootingStop(){
        return Commands.parallel(
            ShootingCommands.Stop(),
            SuperStructureConstants.StopRoller()
        );
    }

    public Command Passing(){
        return Commands.parallel(
            ShootingCommands.Pass(),
            SuperStructureConstants.runHopperRoller()
        );
    }

    

}
