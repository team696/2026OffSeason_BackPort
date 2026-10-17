// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.bases.MotorSubsystem;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class Intake{

  public static class RackAndPinion extends ServoMotorSubsystem<MotorIOTalonFX>{
    private static RackAndPinion RackAndPinion = null;
    public static synchronized RackAndPinion get(){
      if(RackAndPinion == null){
        RackAndPinion = new RackAndPinion();
      }
      return RackAndPinion;
    }

    public static final Setpoint RETRACT = Setpoint.withPositionSetpoint(IntakeConstants.RackAndPinion.kInAngle);
    public static final Setpoint EXTEND = Setpoint.withPositionSetpoint(IntakeConstants.RackAndPinion.kOutAngle);
    public static final Setpoint HALF = Setpoint.withPositionSetpoint(IntakeConstants.RackAndPinion.kInAngle);


    public RackAndPinion(){
      super(
        IntakeConstants.RackAndPinion.getMotorIO(), 
        "Rack and Pinon", 
        IntakeConstants.RackAndPinion.kEpsilonThreshold);
        setCurrentPosition(IntakeConstants.RackAndPinion.kInAngle);

;

    }

    @Override
    public void periodic() {
    super.periodic();
      SmartDashboard.putNumber("Rack and Pinion Pose", this.getPosition().in(Rotations));
      Setpoint target = getSetpoint();
      boolean positionControlled = target != null && target.mode.isPositionControl();
      double targetRotations = positionControlled ? target.baseUnits / (2.0 * Math.PI) : Double.NaN;
      SmartDashboard.putString("Intake/Rack and Pinion/Control Mode", target == null ? "NONE" : target.mode.name());
      SmartDashboard.putBoolean("Intake/Rack and Pinion/Position Control Active", positionControlled);
      SmartDashboard.putNumber("Intake/Rack and Pinion/Target Rotations", targetRotations);
      SmartDashboard.putNumber("Intake/Rack and Pinion/Position Error Rotations",
          targetRotations - getPosition().in(Rotations));
  }

  }

	public static class Rollers extends MotorSubsystem<MotorIOTalonFX>{
		private static Rollers rollers = null;
		public static synchronized Rollers get(){
			if(rollers == null){
				rollers = new Rollers();
			}
			return rollers;
		}

		public static final Setpoint IDLE = Setpoint.COAST;
		public static final Setpoint INTAKE = Setpoint.withVelocitySetpoint(IntakeConstants.Roller.kIntakeSpeed);
		public static final Setpoint OUTAKE = Setpoint.withVelocitySetpoint(IntakeConstants.Roller.kOuttakeSpeed);
    public static final Setpoint SLOW_INTAKE = Setpoint.withVelocitySetpoint(IntakeConstants.Roller.kSlowIntakeSpeed);
		public Rollers(){
			super(IntakeConstants.Roller.getMotorIO(), "Intake Roller");
		}

    @Override
    public void periodic() {
      super.periodic();
      SmartDashboard.putNumber("Intake Roller Velocity", this.getVelocity().in(RotationsPerSecond));
    }

	}

  	public static class Rollers2 extends MotorSubsystem<MotorIOTalonFX>{
		private static Rollers2 rollers = null;
		public static synchronized Rollers2 get(){
			if(rollers == null){
				rollers = new Rollers2();
			}
			return rollers;
		}

		public static final Setpoint IDLE = Setpoint.COAST;
		public static final Setpoint INTAKE = Setpoint.withVelocitySetpoint(IntakeConstants.Roller2.kIntakeSpeed);
		public static final Setpoint OUTAKE = Setpoint.withVelocitySetpoint(IntakeConstants.Roller2.kOuttakeSpeed);
    public static final Setpoint SLOW_INTAKE = Setpoint.withVelocitySetpoint(IntakeConstants.Roller2.kSlowIntakeSpeed);
		public Rollers2(){
			super(IntakeConstants.Roller2.getMotorIO(), "Intake Roller 2");
		}

    @Override
    public void periodic() {
      super.periodic();
      SmartDashboard.putNumber("Intake Roller 2 Velocity", this.getVelocity().in(RotationsPerSecond));
    }

	}

}
