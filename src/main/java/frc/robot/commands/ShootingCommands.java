// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.ShootingSetpoint;
import frc.robot.shooting.ShootingState;
import frc.robot.shooting.motionComp.MotionComp;
import frc.robot.subsystems.FeedRollers.FeedRollers;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

/** Add your docs here. */
public class ShootingCommands {

    public static Command spinUp() {
        return Commands.parallel(
                Shooter.get().followSetpointCommand(
                () -> ShootingSetpoint.mInstance.getShooterSetpoint()
            ),
                Hood.get().followSetpointCommand(
                () -> ShootingSetpoint.mInstance.getHoodSetpoint()
            )
        );
        
    }

    public static Command testspinUp() {
        return Shooter.get().getTestShot()
        .alongWith(Hood.get().getTestShotCommand());

    }

    public static boolean atShooterSpeed() {
        return Shooter.get().spunUp(ShooterConstants.kFeedTolerance);
    }

    public static boolean readyToShoot() {
        return atShooterSpeed();  //TODO: Add Check for alignment to goal angle.
    }
    
    /** Waits until the drum is at its target speed. Put this right before the feed rollers start. */
    public static Command waitUntilShooterAtSpeed() {
        return Commands.waitUntil(ShootingCommands::atShooterSpeed);
    }

    public static Command testShot(){
        return Commands.parallel(
            testspinUp(),
            Commands.sequence(
                waitUntilShooterAtSpeed(),
                FeedRollers.get().setpointCommand(FeedRollers.RUNNING)
            )
        );
    }

    // State is set when the command starts (beforeStarting), not when it's built,
    // otherwise whichever command was constructed last wins for every binding
    public static Command AutoAlgin(){
        return Commands.parallel(
            Shooting.mInstance.getMotionComp().PointToTarget(),
            spinUp()
        ).beforeStarting(() -> Shooting.setState(ShootingState.HUB));
    }

    public static Command shoot(){
        return Commands.parallel(
            Shooting.mInstance.getMotionComp().PointToTarget(),
            spinUp(),
            Commands.run(()->{
                if (ShootingCommands.readyToShoot()) {
                    FeedRollers.get().applySetpoint(FeedRollers.RUNNING);
                } else {
                    FeedRollers.get().applySetpoint(FeedRollers.IDLE);
                }
            }, FeedRollers.get())
        ).beforeStarting(() -> Shooting.setState(ShootingState.HUB))
        .finallyDo(interrupted -> {
            // Apply cleanup directly: an enclosing auto may still own these subsystems.
            FeedRollers.get().applySetpoint(FeedRollers.IDLE);
            Shooter.get().applySetpoint(Shooter.IDLE);
            Hood.get().applySetpoint(Hood.STOW);
        });
    }
    //Shooter.get().spunUp(ShooterConstants.kFeedTolerance)
    /* 
     public static Command shoot(){
        return Commands.parallel(
            Shooting.mInstance.getMotionComp().PointToTarget(),
            spinUp(),
            Commands.sequence(
                Commands.waitUntil(Shooting.mInstance::shotVerified),
                waitUntilShooterAtSpeed(),
                FeedRollers.get().setpointCommand(FeedRollers.RUNNING)
            )
        ).beforeStarting(() -> Shooting.setState(ShootingState.HUB));
    }*/

    public static Command NoVerifyShoot(){
         return Commands.parallel(
            Shooting.mInstance.getMotionComp().PointToTarget(),
            spinUp(),
            Commands.sequence(
                waitUntilShooterAtSpeed(),
                FeedRollers.get().setpointCommand(FeedRollers.RUNNING)
            )
        ).beforeStarting(() -> Shooting.setState(ShootingState.HUB));
    }



    // side is NEUTRAL_FERRY_LEFT or NEUTRAL_FERRY_RIGHT
    public static Command Pass(ShootingState side){
        return Commands.parallel(
            Shooting.mInstance.getMotionComp().PointToTarget(),
            spinUp(),
            Commands.sequence(
                Commands.waitUntil(Shooting.mInstance::shotVerified),
                waitUntilShooterAtSpeed(),
                FeedRollers.get().setpointCommand(FeedRollers.RUNNING)
            )
        ).beforeStarting(() -> Shooting.setState(side));
    }

    public static Command Stop(){
        return Commands.parallel(
            Shooter.get().setpointCommand(Shooter.IDLE),
            Hood.get().setpointCommand(Hood.STOW),
            FeedRollers.get().setpointCommand(FeedRollers.IDLE)
        );
    }
}
