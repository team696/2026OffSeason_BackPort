// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Shooter extends FlywheelMotorSubsystem<MotorIOTalonFX> {

    private static Shooter shooter = null;
    public static synchronized Shooter get(){
      if(shooter == null){
        shooter = new Shooter();
      }
      return shooter;
    }

  

  //testshot
  //idle
  //ferry
  //shot
  public static Setpoint TESTSHOT = Setpoint.withVelocitySetpoint(RotationsPerSecond.of(ShooterConstants.testShotSpeed.getAsDouble()));
  public static Setpoint IDLE = Setpoint.COAST;
  public static Setpoint FERRY = Setpoint.withVelocitySetpoint(ShooterConstants.FerrySpeed);
  public static Setpoint SHOT = Setpoint.withVelocitySetpoint(ShooterConstants.ShotSpeed);


  /** Creates a new Shooter. */
  private Shooter() {
    super(
      ShooterConstants.getMotorIO(),
      "Shooter",
      ShooterConstants.epsilonThreshold,
      Seconds.of(0)
    );

  }

  public Command getTestShot() {
    return followSetpointCommand(()->
        Setpoint.withVelocitySetpoint(
            RotationsPerSecond.of(ShooterConstants.testShotSpeed.getAsDouble())
        )
    );
  }

    @Override
      public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Shooter speed", Shooter.get().getVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("Shooter Setpoint", Shooter.get().getSetpoint().baseUnits / 2 / Math.PI);

      }
}
