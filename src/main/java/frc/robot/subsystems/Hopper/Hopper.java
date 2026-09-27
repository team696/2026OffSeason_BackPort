// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.bases.MotorSubsystem;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.robot.subsystems.Shooter.Shooter;
import frc.lib.io.MotorIO.Setpoint;

public class Hopper {

  public static class Rollers extends MotorSubsystem<MotorIOTalonFX>{

    private static Rollers roller = null;
    public static synchronized Rollers get(){
      if(roller == null){
        roller = new Rollers();
      }
      return roller;
    }


    public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(HopperConstants.Rollers.kIntakeVoltage);
    public static final Setpoint SLOW_INTAKE = Setpoint.withVoltageSetpoint(HopperConstants.Rollers.kSlowIntakeVoltage);
    public static final Setpoint OUTAKE = Setpoint.withVoltageSetpoint(HopperConstants.Rollers.kOuttakeVoltage);
    public static final Setpoint IDLE = Setpoint.COAST;

    public Rollers(){
      super(HopperConstants.Rollers.getMotorIO(), "Hopper Rollers");
    }

      @Override
      public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Hopper Speed", this.getVelocity().in(RotationsPerSecond));
      }
    

  }

  // public static class Upwards extends ServoMotorSubsystem<MotorIOTalonFX> {
  //   public static final Setpoint MOVE_HIGH = Setpoint.withPositionSetpoint(HopperConstants.Upwards.HighPosAngle);
  //   public static final Setpoint MOVE_LOW = Setpoint.withPositionSetpoint(HopperConstants.Upwards.LowPosAngle);


  //   private static Upwards m_hopper = null;
  //   public static Upwards get() {
  //       if (m_hopper == null) {
  //           m_hopper = new Upwards();
  //       }
  //       return m_hopper;
  //   }

  //   public Upwards() {
  //       //tuing it for fat chuds
  //       super(HopperConstants.Upwards.getMotorIO(), "Upwards Hopper", HopperConstants.Upwards.kEpsilonThreshold);
  //       setCurrentPosition(HopperConstants.Upwards.LowPosAngle); //by default should be low
  //   }
  // }

}
