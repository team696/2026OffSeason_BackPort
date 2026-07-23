// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FeedRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class FeedRollers extends MotorSubsystem<MotorIOTalonFX> {
  /** Creates a new FeedRollers. */

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
