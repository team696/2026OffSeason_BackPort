// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Shooter extends FlywheelMotorSubsystem<MotorIOTalonFX> {

  //testshot
  //idle
  //ferry
  //shot
  public static Setpoint TESTSHOT = Setpoint.withVelocitySetpoint(ShooterConstants.TestShotSpeed);
  public static Setpoint IDLE = Setpoint.withVelocitySetpoint(ShooterConstants.IdleSpeed);
  public static Setpoint FERRY = Setpoint.withVelocitySetpoint(ShooterConstants.FerrySpeed);
  public static Setpoint SHOT = Setpoint.withVelocitySetpoint(ShooterConstants.ShotSpeed);


  /** Creates a new Shooter. */
  public Shooter() {
    super(
      ShooterConstants.getMotorIO(),
      "Shooter",
      ShooterConstants.epsilonThreshold,
      Seconds.of(0)
    );
  }
}
