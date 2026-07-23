// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.lib.sim.PivotSimHelper;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class IntakeConstants {

    public class Pivot{
        public static final Angle kDeployAngle = Units.Degrees.of(0);
        public static final Angle kStowAngle = Units.Degrees.of(130);
        public static final Angle kPartiallyin = Units.Degrees.of(80);

	    public static final Voltage kSimStaticFrictionVoltage = Units.Volts.of(0.0);
	    public static final Voltage kSimKineticFrictionVoltage = Units.Volts.of(0.0);
	    public static final AngularVelocity kSimStictionVelocity = Units.DegreesPerSecond.of(1.0);
	    public static final double kSimViscousFrictionVoltsPerRadPerSec = 0.38;

        public static double kGearing = 15/1; //15 to 1 gear ratio;

        public static final Distance Arm_distance = Units.Inches.of(12.804);
        public static final Angle kEpsilonThreshold = Units.Degrees.of(12.0);

        //There is propbably a lot more

        public static TalonFXConfiguration getTalonFXConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kP = 1;
            config.Slot0.kI = 0;
            config.Slot0.kD = 0;

            config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

            config.MotionMagic.MotionMagicCruiseVelocity = 7.0; // rps
            config.MotionMagic.MotionMagicAcceleration = 15.0; //rps^2

            config.Voltage.PeakForwardVoltage = 12.0; //Prevent going over 12 volts 
            config.Voltage.PeakReverseVoltage = -12.0; 

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
		    config.CurrentLimits.SupplyCurrentLimit = 40.0;
		    config.CurrentLimits.SupplyCurrentLowerLimit = 25.0;
		    config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.CurrentLimits.StatorCurrentLimit = 60.0;
		    config.CurrentLimits.StatorCurrentLimitEnable = true;

            config.Feedback.SensorToMechanismRatio = kGearing;

            
		    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kStowAngle.in(Units.Rotations);

            return config;

        }

        public static MotorIOTalonFXConfig getIOConfig(){
            MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
            config.mainConfig = getTalonFXConfig();
            config.mainID = Ports.INTAKE_PIVOT.id;
            config.mainBus = Ports.INTAKE_PIVOT.bus;
            config.time = Units.Seconds;
            config.unit = Units.Degrees;    
            return config;
        }

        public static MotorIOTalonFX geMotorIO(){
            if(Robot.isReal()){
                return new MotorIOTalonFX(getIOConfig());
            }
            else{
                return new MotorIOTalonFXSim(getIOConfig(), new PivotSimHelper(getSimConstants(), 
                    kSimStaticFrictionVoltage, 
                    kSimKineticFrictionVoltage, 
                    kSimStictionVelocity, 
                    kSimViscousFrictionVoltsPerRadPerSec));
            }
        }

    public static PivotSimConstants getSimConstants() {
		PivotSimConstants simConstants = new PivotSimConstants();
		simConstants.gearing = kGearing;
		simConstants.armLength = Arm_distance;
		simConstants.momentOfInertia = Units.KilogramSquareMeters.of(0.2224061366);
		simConstants.motor = DCMotor.getKrakenX44Foc(1);
		simConstants.mechanismMaxHardStop = kStowAngle;
		simConstants.mechanismMinHardStop = kDeployAngle;
		simConstants.simGravity = false;
		simConstants.mechanismStartPos = kStowAngle;

		return simConstants;
	}

    public static ServoHomingConfig getServoHomingConfig(){
        ServoHomingConfig config = new ServoHomingConfig();
        config.kHomePosition = kStowAngle;
        config.kHomingTimeout = Units.Seconds.of(2.0);
        config.kHomingVoltage = Units.Volts.of(2);
        config.kSetHomedVelocity = Units.DegreesPerSecond.of(1.0);
        return config;
    }

            
    }

    public class Roller{
        public static final double kGearing = 1;

        public static final Voltage kIdleVoltage = Units.Volts.of(0.0);
        public static final Voltage kIntakeVoltage = Units.Volts.of(12.0);
        public static final Voltage kOuttakeVoltage = Units.Volts.of(-12.0);

        public static TalonFXConfiguration getTaloFXConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
            config.CurrentLimits.StatorCurrentLimit = 80;

            config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
            config.CurrentLimits.SupplyCurrentLimit = 50.0;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Voltage.PeakForwardVoltage = 12.0;
	        config.Voltage.PeakReverseVoltage = -12.0;

            config.Feedback.SensorToMechanismRatio = kGearing;


            return config;
        }

        public static MotorIOTalonFXConfig getIOConfig(){
            MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
            config.mainConfig = getTaloFXConfig();
            config.mainID = Ports.INTAKE_ROLLERS_MAIN.id;
            config.mainBus = Ports.INTAKE_ROLLERS_MAIN.bus;
            config.followerConfig = getTaloFXConfig();
            config.followerAlignment = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
            config.followerIDs = new int[] {Ports.INTAKE_ROLLERS_FOLLOWER.id};
            config.followerBuses = new CANBus[] {Ports.INTAKE_ROLLERS_FOLLOWER.bus};
            return config;
        }

        public static MotorIOTalonFX getMotorIO(){
            if(Robot.isReal()){
                return new MotorIOTalonFX(getIOConfig());
            }else{
                return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
            }
        }

        public static RollerSimConstants getSimConstants() {
            RollerSimConstants simConstants = new RollerSimConstants();

            simConstants.motor = DCMotor.getKrakenX60(1);
            simConstants.gearing = kGearing;
            simConstants.momentOfInertia = 0.0001618297;

            return simConstants;
        }





    }


}
