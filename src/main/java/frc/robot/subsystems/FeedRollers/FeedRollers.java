// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FeedRollers;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class FeedRollers extends MotorSubsystem<MotorIOTalonFX> {
  /** Creates a new FeedRollers. */

    private static FeedRollers roller = null;
    public static synchronized FeedRollers get(){
      if(roller == null){
        roller = new FeedRollers();
      }
      return roller;
    }

  public static Setpoint IDLE = Setpoint.withVelocitySetpoint(FeedRollersConstants.IdleSpeed);
  public static Setpoint RUNNING = Setpoint.withVelocitySetpoint(FeedRollersConstants.RunningSpeed);

  public FeedRollers() {
    super(FeedRollersConstants.getMotorIO(), "FeedRollers");
    applySetpoint(IDLE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
