// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.lib.sim.PivotSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.lib.util.tunablenumbers.TunableNumber;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class HopperConstants {
// public class Upwards {
//         //this should be defiedn in the Ports enum but for the ehhh later

//         //forgot if we have one or 2 motors on it but eh i can always add it in later

//         public static Distance LowPos = Units.Meter.of(0);
//         public static Distance HighPos = Units.Meter.of(1);
//         //stuff for 

//         //TODO: change this to the real value
//         public static final Distance kRadius = Units.Inches.of(34.0 / 20.0).div(2.0);
//         public static final DistanceAngleConverter converter = new DistanceAngleConverter(kRadius);

//         public static Angle LowPosAngle = converter.toAngle(LowPos);
//         public static Angle HighPosAngle = converter.toAngle(HighPos);

//         public static final double kGearingRotate  = 1;

//         public static final Angle kEpsilonThreshold = Units.Degree.of(5);

//         public static LinearSimConstants getSimConstants() {
//             LinearSimConstants constants = new LinearSimConstants();

//             constants.gearing = kGearingRotate;
//             constants.minHeight = LowPos;
//             constants.maxHeight = HighPos;
//             constants.simGravity = false;
//             constants.startingHeight = LowPos;
//             constants.motor = DCMotor.getKrakenX60(2);
//             constants.carriageMass = Units.Pounds.of(8);
//             constants.converter = converter;

//             return constants;
//         }

//         public static MotorIOTalonFXConfig getConfig() {
//             MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();

//             config.mainBus = Ports.HOPPER_UPWARDS_MAIN.bus; //no way we getting vores
//             config.mainID = Ports.HOPPER_UPWARDS_MAIN.id;

//             //config the submisive one
//             config.followerAlignment = new MotorAlignmentValue[] { MotorAlignmentValue.Opposed };
//             config.followerBuses = new CANBus[] { Ports.HOPPER_UPWARDS_FOLLOWER.bus };
//             config.followerIDs = new int[] { Ports.HOPPER_UPWARDS_FOLLOWER.id };

//             config.time = Units.Seconds;
//             config.unit = Units.Degrees;

//             //just some standard power config
//             TalonFXConfiguration config_talon = new TalonFXConfiguration();
//             config_talon.Slot0.kP = 1;
//             config_talon.Slot0.kI = 0;
//             config_talon.Slot0.kD = 0;

//             config_talon.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
//             config_talon.CurrentLimits.StatorCurrentLimit = 90.0;

//             config_talon.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
//             config_talon.CurrentLimits.SupplyCurrentLimit = 45.0;

//             config_talon.Feedback.SensorToMechanismRatio = kGearingRotate;

//             //apply the talon configs
//             config.mainConfig = config_talon;
//             config.followerConfig = config_talon;
//             return config;
//         }


//         public static MotorIOTalonFX getMotorIO() {
//             if (Robot.isReal()) {
//                 return new MotorIOTalonFX(getConfig());
//             } else {
//                 return new MotorIOTalonFXSim(getConfig(), new LinearSim(getSimConstants()));
//             }
//         }

//     }


public class HopperPivot{
    // 20:1 planetary, then a 20T pulley driving an 18T pulley, then a 24T gear (on the 18T pulley) driving a 60T gear
    public static final double kGearing = 20.0 * (18.0 / 20.0) * (60.0 / 24.0);
    public static final Angle kEpsilon = Degree.of(0.1);

    public static final Angle kLowestAngle = Units.Degrees.of(0.0);
    public static final Angle kHighest = Units.Degrees.of(5.0);
    public static final Angle kUpangle = Units.Degrees.of(-15.0);

    public static final AngularVelocity kUpCruiseVelocity = Units.DegreesPerSecond.of(30.0);
    public static final AngularVelocity kDownCruiseVelocity = Units.DegreesPerSecond.of(5.0);

    public static final TunableNumber testAngle = new TunableNumber("Hopper Pivot Test Angle", 0.0);

    public static final Mass kSimMass = Units.Pounds.of(8.0);

    public static final TalonFXConfiguration getTalonFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

       
        config.Slot2.kP = 360;
        config.Slot2.kI = 0;
        config.Slot2.kD = 0;

        config.Slot2.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot2.kG = 0.25;
        // Volts to break static friction; tune by raising voltage slowly until it just starts moving.
        config.Slot2.kS = 0.1;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.MotionMagic.MotionMagicCruiseVelocity = kDownCruiseVelocity.in(Units.RotationsPerSecond);
        config.MotionMagic.MotionMagicExpo_kV = 0.12;
        config.MotionMagic.MotionMagicExpo_kA = 0.1;


        //TODO
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 90.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 45.0;

        config.Feedback.SensorToMechanismRatio = kGearing;

        return config;
    }


    public static MotorIOTalonFXConfig getIOConfig(){
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.mainConfig = getTalonFXConfig();
        config.mainID = Ports.HOPPER_PIVOT.id;
        config.mainBus = Ports.HOPPER_PIVOT.bus;
        config.time = Units.Seconds;
        config.unit = Units.Degrees;
        return config;
    }

    public static MotorIOTalonFX getMotorIO(){
        if(Robot.isReal()){
            return new MotorIOTalonFX(getIOConfig());
        }
        else{
            return new MotorIOTalonFXSim(getIOConfig(), new PivotSim(getSimConstants()));
        }
    }

    public static PivotSimConstants getSimConstants(){
        PivotSimConstants constants = new PivotSimConstants();
		constants.gearing = kGearing;
		constants.motor = DCMotor.getKrakenX60(1);
        constants.armLength = Inch.of(19.5);
        constants.momentOfInertia = Units.KilogramSquareMeters.of(
            SingleJointedArmSim.estimateMOI(constants.armLength.in(Units.Meters), kSimMass.in(Units.Kilograms)));
        constants.mechanismMinHardStop = kLowestAngle;
        constants.mechanismMaxHardStop = kHighest;
        constants.simGravity = false;
        constants.mechanismStartPos = kLowestAngle;
		return constants;
    }
}

public class Rollers{    
        
    public static final double kGearing  = 1;

    public static final Voltage kIntakeVoltage = Volts.of(6.0); 
	public static final Voltage kSlowIntakeVoltage = Volts.of(1.5); 
	public static final Voltage kOuttakeVoltage = Volts.of(-8.0); 
	public static final Voltage kIdleVoltage = Volts.of(0.0); 
	public static final Voltage kPreloadSpeed = Volts.of(8.0); 

	public static final Current kCurrentThreshold = Amps.of(50); 


    public static final TalonFXConfiguration getTalonFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 40.0;

        config.Feedback.SensorToMechanismRatio = kGearing;

        return config;
    }

    public static MotorIOTalonFXConfig getIOConfig(){
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();
        config.mainConfig = getTalonFXConfig();
        config.mainID = Ports.HOPPER_ROLLERS.id;
        config.mainBus = Ports.HOPPER_ROLLERS.bus;
        config.time = Units.Seconds;
        config.unit = Units.Degrees;
        return config;
    }

    public static MotorIOTalonFX getMotorIO(){
        if(Robot.isReal()){
            return new MotorIOTalonFX(getIOConfig());
        }
        else{
            return new MotorIOTalonFXSim(getIOConfig(), new RollerSim(getSimConstants()));
        }
    }

    public static RollerSimConstants getSimConstants(){
        RollerSimConstants constants = new RollerSimConstants();
		constants.gearing = kGearing;
		constants.motor = DCMotor.getKrakenX60(1);
		constants.momentOfInertia = 0.0000175584;
		return constants;
    }
}

    
}
