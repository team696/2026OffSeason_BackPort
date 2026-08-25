// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.ShootingSetpoint;
import frc.robot.shooting.ShootingState;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Shooter.Shooter;

/** Add your docs here. */
public class ShootingCommands {

    public static Command spinUp() {
        return Commands.parallel(
            Shooter.get().run(() -> 
                Shooter.get().setpointCommand(ShootingSetpoint.mInstance.getShooterSetpoint())),
            Hood.get().run(() -> 
                Hood.get().setpointCommand(ShootingSetpoint.mInstance.getHoodSetpoint()))
        );
    }

    public static Command testspinUp() {
        Hood.get();
        Shooter.get();
        return Commands.parallel(
            Shooter.get().run(() -> 
                Shooter.get().setpointCommand(Shooter.TESTSHOT)),
            Hood.get().run(() -> 
                Hood.get().setpointCommand(Hood.TEST_ANGLE))
        );
    }

    
    public static Command testShot(){
        return Commands.parallel(
            testspinUp(),
            FeedRollers.get().setpointCommand(FeedRollers.RUNNING)
        );
    }

    public static Command AutoAlgin(){
        Shooting.setState(ShootingState.HUB);
        return Commands.run(()->{Shooting.mInstance.getMotionComp().PointToTarget();});
    }

    public static Command shoot(){
        Shooting.setState(ShootingState.HUB);
        return Commands.parallel(
            spinUp(),
            Commands.sequence(
                Commands.waitUntil(Shooting.mInstance::shotVerified),
                FeedRollers.get().setpointCommand(FeedRollers.RUNNING)
            )
        );
    }

    public static Command Pass(){
        Shooting.setState(ShootingState.NEUTRAL_FERRY);
        return Commands.parallel(
            Shooting.mInstance.getMotionComp().PointToTarget(),
            spinUp(),
            Commands.sequence(
            Commands.waitUntil(Shooting.mInstance::shotVerified),
            FeedRollers.get().setpointCommand(FeedRollers.RUNNING))
        );
    }

    public static Command Stop(){
        return Commands.parallel(
            Shooter.get().setpointCommand(Shooter.IDLE),
            FeedRollers.get().setpointCommand(FeedRollers.IDLE)
        );
    }
}
