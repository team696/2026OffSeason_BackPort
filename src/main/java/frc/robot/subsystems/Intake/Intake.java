// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.MotorSubsystem;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Mode;
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
    public static final Setpoint HALF = Setpoint.withPositionSetpoint(IntakeConstants.RackAndPinion.kHalfAngle);


    public RackAndPinion(){
      super(
        IntakeConstants.RackAndPinion.getMotorIO(),
        "Rack and Pinon",
        IntakeConstants.RackAndPinion.kEpsilonThreshold);
        // Assign the boot position as 0 inches without driving into the hard stop.
        setCurrentPosition(IntakeConstants.RackAndPinion.kHardStopAngle);

;

    }

    /** Explicit operator homing can be repeated after a lost reference or failed attempt. */
    public Command zeroCommand() {
      var config = IntakeConstants.RackAndPinion.getZeroConfig();
      // Reject the recorded mid-travel stall instead of redefining that location as zero.
      // This still relies on the old relative reference; a home switch is needed for absolute proof.
      config.kMaxPreZeroPositionError = config.kConfirmTolerance;
      config.kMaxFeedbackAge = Seconds.of(0.1);
      return zeroAgainstHardStopCommand(config, RETRACT);
    }

    @Override
    public void periodic() {
    super.periodic();
      SmartDashboard.putNumber("Rack and Pinion Pose", this.getPosition().in(Rotations));
      SmartDashboard.putNumber("Rack and Pinion Desired", this.getSetpoint().baseUnits / 2 / Math.PI);
      Setpoint target = getSetpoint();
      boolean positionControlled = target != null && target.mode.isPositionControl();
      double targetRotations = positionControlled ? target.baseUnits / (2.0 * Math.PI) : Double.NaN;
      String controlMode = target == null ? "NONE" : target.mode.name();
      double requestedVoltage = target != null && target.mode == Mode.VOLTAGE ? target.baseUnits : Double.NaN;
      SmartDashboard.putString("Intake/Rack and Pinion/Control Mode", controlMode);
      SmartDashboard.putBoolean("Intake/Rack and Pinion/Position Control Active", positionControlled);
      SmartDashboard.putNumber("Intake/Rack and Pinion/Target Rotations", targetRotations);
      SmartDashboard.putNumber("Intake/Rack and Pinion/Position Error Rotations",
          targetRotations - getPosition().in(Rotations));
      SmartDashboard.putString("Intake/Rack and Pinion/Homing State", getZeroingState());
      SmartDashboard.putString("Intake/Rack and Pinion/Homing Failure", getZeroingFailure());
      SmartDashboard.putBoolean("Intake/Rack and Pinion/Has Verified Home", hasZeroed());
      SmartDashboard.putNumber("Intake/Rack and Pinion/Homing Attempts", getZeroingAttempts());
      SmartDashboard.putNumber("Intake/Rack and Pinion/Last Prezero Position Rotations",
          getLastPreZeroPositionRadians() / (2.0 * Math.PI));
      // Already cached by MotorIO: these reads do not request extra CAN updates.
      SmartDashboard.putNumber("Intake/Rack and Pinion/Motor Voltage", getMotorVoltage().in(Volts));
      SmartDashboard.putNumber("Intake/Rack and Pinion/Stator Current Amps", getStatorCurrent().in(Amps));
      SmartDashboard.putNumber("Intake/Rack and Pinion/Supply Current Amps", getSupplyCurrent().in(Amps));
      SmartDashboard.putNumber("Intake/Rack and Pinion/Requested Voltage", requestedVoltage);
      // Native motor signals already go to HOOT. Record the command beside them so a
      // partial WPILOG or a different clock cannot hide the target during a loaded stop.
      SignalLogger.writeDouble("Intake/Rack and Pinion/FPGA Seconds", Timer.getFPGATimestamp(), "s");
      SignalLogger.writeString("Intake/Rack and Pinion/Control Mode", controlMode);
      SignalLogger.writeDouble("Intake/Rack and Pinion/Target Rotations", targetRotations, "rot");
      SignalLogger.writeDouble("Intake/Rack and Pinion/Requested Voltage", requestedVoltage, "V");
      SignalLogger.writeString("Intake/Rack and Pinion/Homing State", getZeroingState());
      SignalLogger.writeString("Intake/Rack and Pinion/Homing Failure", getZeroingFailure());
      SignalLogger.writeInteger("Intake/Rack and Pinion/Homing Attempts", getZeroingAttempts());
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
