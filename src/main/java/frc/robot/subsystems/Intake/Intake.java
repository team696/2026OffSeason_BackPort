// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.lib.bases.MotorSubsystem;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class Intake{

    public static class Pivot extends ServoMotorSubsystem<MotorIOTalonFX>{

		private static Pivot pivot = null;
		public static synchronized Pivot get(){
			if(pivot == null){
				pivot = new Pivot();
			}
			return pivot;
		}

      	public static final Setpoint STOWED = Setpoint.withPositionSetpoint(IntakeConstants.Pivot.kStowAngle);
	    public static final Setpoint DEPLOYED =
			    Setpoint.withPositionSetpoint(IntakeConstants.Pivot.kDeployAngle);
	    public static final Setpoint PARTIAL = Setpoint.withPositionSetpoint(IntakeConstants.Pivot.kPartiallyin);
		private Pivot() {
				super(IntakeConstants.Pivot.geMotorIO(), "Intake Deploy", IntakeConstants.Pivot.kEpsilonThreshold,
					IntakeConstants.Pivot.getServoHomingConfig()
					);
				setCurrentPosition(IntakeConstants.Pivot.kStowAngle);
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

		public static final Setpoint IDLE = Setpoint.withNeutralSetpoint();
		public static final Setpoint INTAKE = Setpoint.withVoltageSetpoint(IntakeConstants.Roller.kIntakeVoltage);
		public static final Setpoint OUTAKE = Setpoint.withVoltageSetpoint(IntakeConstants.Roller.kOuttakeVoltage);
		public Rollers(){
			super(IntakeConstants.Roller.getMotorIO(), "Intake Roller");
		}


	}

}
