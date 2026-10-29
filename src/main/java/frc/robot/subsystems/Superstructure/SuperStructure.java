// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Superstructure;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.commands.ShootingCommands;
import frc.robot.shooting.ShootingState;
import frc.lib.io.MotorIO.Setpoint;

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
            Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.HALF),
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

        return shootingWithIntake(ShootingCommands.shoot());
    }

    public static Command ManualShooting(){
        return shootingWithIntake(ShootingCommands.manualShoot());
    }

    static double time = 0;

    private static Command shootingWithIntake(Command shot){
        return Commands.parallel(
            shot,
            Commands.parallel(
                Intake.RackAndPinion.get().startRun(
                     ()->{
                        time = Timer.getFPGATimestamp();
                     }, ()->{
                        Intake.RackAndPinion.get().applySetpoint( 
                            Setpoint.withPositionSetpoint( 
                                IntakeConstants.RackAndPinion.converter.toAngle(
                                    Units.Inches.of(
                                        (1-Math.min(1, (Timer.getFPGATimestamp() - time) / 3)) * 3.5 + .3
                                    )) ));
                    }),
                //Intake.RackAndPinion.get().setpointCommand(Intake.RackAndPinion.HALF),
                Intake.Rollers.get().setpointCommand(Intake.Rollers.INTAKE),
                Intake.Rollers2.get().setpointCommand(Intake.Rollers2.INTAKE),
                //SuperStructureConstants.StopIntakeRollers(),  //stop intake rollers during shooting AST 20261020
                SuperStructureConstants.runHopperRoller()
                )
            ).finallyDo(interrupted -> {
                Intake.Rollers.get().applySetpoint(Intake.Rollers.IDLE);
                Intake.Rollers2.get().applySetpoint(Intake.Rollers2.IDLE);
                Intake.Rollers.get().setpointCommand(Intake.Rollers.IDLE);
                Intake.Rollers2.get().setpointCommand(Intake.Rollers2.IDLE);
            });
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
            SuperStructureConstants.StopRoller(),
            SuperStructureConstants.StopIntakeRollers()
        );
    }

    public static Command Passing(ShootingState side){
        return Commands.parallel(
            Intake.RackAndPinion.get().startRun(
                     ()->{
                        time = Timer.getFPGATimestamp();
                     }, ()->{
                        Intake.RackAndPinion.get().applySetpoint( 
                            Setpoint.withPositionSetpoint( 
                                IntakeConstants.RackAndPinion.converter.toAngle(
                                    Units.Inches.of(
                                        (1-Math.min(1, (Timer.getFPGATimestamp() - time) / 3)) * 3.5 + .3
                                    )) ));
                    }),
            ShootingCommands.Pass(side),
            SuperStructureConstants.runHopperRoller()
        );
    }
}
