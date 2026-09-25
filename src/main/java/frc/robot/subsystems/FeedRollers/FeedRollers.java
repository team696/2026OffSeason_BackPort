// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FeedRollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.UnaryOperator;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIO.Mode;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.shooting.Shooting;
import frc.robot.shooting.ShootingSetpoint;

public class FeedRollers extends FlywheelMotorSubsystem<MotorIOTalonFX> {
  /** Creates a new FeedRollers. */

    private static FeedRollers roller = null;
    public static synchronized FeedRollers get(){
      if(roller == null){
        roller = new FeedRollers();
      }
      return roller;
    }

  public static Setpoint IDLE = Setpoint.withVelocitySetpoint(FeedRollersConstants.IdleSpeed , 1);
  //public static Setpoint RUNNING;
    public static Command RUNNING = get().followSetpointCommand(() -> {
      AngularVelocity vel = Shooting.getState().getRegression().getShooterVeloicty(ShootingSetpoint.mInstance.getLookAheadDistance()).times(FeedRollersConstants.PercentOfLUT);
      return Setpoint.withVelocitySetpoint(vel, 1);
    }); 
  
    public FeedRollers() {
    super(
      FeedRollersConstants.getMotorIO(),
      "FeedRollers",
      FeedRollersConstants.epsilonThreshold,
      FeedRollersConstants.debounder
    );

    //define custom setpoint RUNNING
    //dfiene the unuary opprator first

    //Setpoint running = new Setpoint();
    applySetpoint(IDLE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
