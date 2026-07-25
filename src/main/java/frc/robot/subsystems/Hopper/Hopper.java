// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import frc.lib.bases.MotorSubsystem;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
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
    public static final Setpoint IDLE = Setpoint.withVoltageSetpoint(HopperConstants.Rollers.kIdleVoltage);

    public Rollers(){
      super(HopperConstants.Rollers.getMotorIO(), "Hopper Rollers");
    }

  }

  public static class RackAndPinion extends ServoMotorSubsystem<MotorIOTalonFX>{
    private static RackAndPinion RackAndPinion = null;
    public static synchronized RackAndPinion get(){
      if(RackAndPinion == null){
        RackAndPinion = new RackAndPinion();
      }
      return RackAndPinion;
    }

    public static final Setpoint RETRACT = Setpoint.withPositionSetpoint(HopperConstants.RackAndPinion.kInAngle);
    public static final Setpoint EXTEND = Setpoint.withPositionSetpoint(HopperConstants.RackAndPinion.kOutAngle);


    public RackAndPinion(){
      super(
        HopperConstants.RackAndPinion.getMotorIO(), 
        "Rack and Pinon", 
        HopperConstants.RackAndPinion.kEpsilonThreshold);
        setCurrentPosition(HopperConstants.RackAndPinion.kInAngle);
    }



  }

  public static class Upwards extends ServoMotorSubsystem<MotorIOTalonFX> {
    public static final Setpoint MOVE_HIGH = Setpoint.withPositionSetpoint(HopperConstants.Upwards.HighPosAngle);
    public static final Setpoint MOVE_LOW = Setpoint.withPositionSetpoint(HopperConstants.Upwards.LowPosAngle);


    private static Upwards m_hopper = null;
    public static Upwards get() {
        if (m_hopper == null) {
            m_hopper = new Upwards();
        }
        return m_hopper;
    }

    public Upwards() {
        //tuing it for fat chuds
        super(HopperConstants.Upwards.getMotorIO(), "Upwards Hopper", HopperConstants.Upwards.kEpsilonThreshold);
        setCurrentPosition(HopperConstants.Upwards.LowPosAngle); //by default should be low
    }
  }

}
