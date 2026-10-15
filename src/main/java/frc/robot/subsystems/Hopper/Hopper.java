// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.bases.MotorSubsystem;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.subsystems.Hopper.HopperConstants.HopperPivot;

public class Hopper {

  public static class Pivot extends ServoMotorSubsystem<MotorIOTalonFX> {
    private static Pivot pivot = null;
    public static synchronized Pivot get(){
      if(pivot == null){
        pivot = new Pivot();
      }
      return pivot;
    }

    public Pivot(){
      super(HopperConstants.HopperPivot.getMotorIO(), "Hopper Pivot", HopperPivot.kEpsilon);
    }

    public static final Setpoint UP = Setpoint.withPositionVelocitySetpoint(HopperPivot.kHighest, HopperPivot.kUpCruiseVelocity);
    public static final Setpoint ZERO = Setpoint.withPositionVelocitySetpoint(HopperPivot.kLowestAngle, HopperPivot.kDownCruiseVelocity);
    public static final Setpoint STOP = Setpoint.withNeutralSetpoint();
    public static final Setpoint NOVOLTAGE = Setpoint.COAST;

    // Goes to whatever "Hopper Pivot Test Angle" is set to on the dashboard (degrees)
    public Command getTestAngleCommand() {
      return followSetpointCommand(() ->
          Setpoint.withPositionSetpoint(Units.Degrees.of(HopperConstants.HopperPivot.testAngle.getAsDouble()))
      );
    }

      @Override
      public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Hopper Pivot Location", this.getPosition().in(Rotation));
      } 
  }

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
