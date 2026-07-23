// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.system.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.LinearSim;
import frc.lib.sim.RollerSim;
import frc.lib.sim.LinearSim.LinearSimConstants;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.lib.util.Util;
import frc.lib.util.Util.DistanceAngleConverter;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotConstants;

/** Add your docs here. */
public class HopperConstants {
public class Upwards {
        //this should be defiedn in the Ports enum but for the ehhh later

        //forgot if we have one or 2 motors on it but eh i can always add it in later

        public static Distance LowPos = Units.Meter.of(0);
        public static Distance HighPos = Units.Meter.of(1);
        //stuff for 

        //TODO: change this to the real value
        public static final Distance kRadius = Units.Inches.of(34.0 / 20.0).div(2.0);
        public static final DistanceAngleConverter converter = new DistanceAngleConverter(kRadius);

        public static Angle LowPosAngle = converter.toAngle(LowPos);
        public static Angle HighPosAngle = converter.toAngle(HighPos);

        public static final double kGearingRotate  = 1;

        public static final Angle kEpsilonThreshold = Units.Degree.of(5);

        public static LinearSimConstants getSimConstants() {
            LinearSimConstants constants = new LinearSimConstants();

            constants.gearing = kGearingRotate;
            constants.minHeight = LowPos;
            constants.maxHeight = HighPos;
            constants.simGravity = false;
            constants.startingHeight = LowPos;
            constants.motor = DCMotor.getKrakenX60(2);
            constants.carriageMass = Units.Pounds.of(8);
            constants.converter = converter;

            return constants;
        }

        public static MotorIOTalonFXConfig getConfig() {
            MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();

            config.mainBus = Ports.HOPPER_UPWARDS_MAIN.bus; //no way we getting vores
            config.mainID = Ports.HOPPER_UPWARDS_MAIN.id;

            //config the submisive one
            config.followerAlignment = new MotorAlignmentValue[] { MotorAlignmentValue.Opposed };
            config.followerBuses = new CANBus[] { Ports.HOPPER_UPWARDS_FOLLOWER.bus };
            config.followerIDs = new int[] { Ports.HOPPER_UPWARDS_FOLLOWER.id };

            config.time = Units.Seconds;
            config.unit = Units.Degrees;

            //just some standard power config
            TalonFXConfiguration config_talon = new TalonFXConfiguration();
            config_talon.Slot0.kP = 1;
            config_talon.Slot0.kI = 0;
            config_talon.Slot0.kD = 0;

            config_talon.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
            config_talon.CurrentLimits.StatorCurrentLimit = 90.0;

            config_talon.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
            config_talon.CurrentLimits.SupplyCurrentLimit = 45.0;

            config_talon.Feedback.SensorToMechanismRatio = kGearingRotate;

            //apply the talon configs
            config.mainConfig = config_talon;
            config.followerConfig = config_talon;
            return config;
        }


        public static MotorIOTalonFX getMotorIO() {
            if (Robot.isReal()) {
                return new MotorIOTalonFX(getConfig());
            } else {
                return new MotorIOTalonFXSim(getConfig(), new LinearSim(getSimConstants()));
            }
        }

    }


public class Rollers{    
        
    public static final double kGearing  = 1;

    public static final Voltage kIntakeVoltage = Volts.of(12.0); 
	public static final Voltage kSlowIntakeVoltage = Volts.of(3.0); 
	public static final Voltage kOuttakeVoltage = Volts.of(-12.0); 
	public static final Voltage kIdleVoltage = Volts.of(0.0); 
	public static final Voltage kPreloadSpeed = Volts.of(3.0); 

	public static final Current kCurrentThreshold = Amps.of(50); 


    public static final TalonFXConfiguration getTalonFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

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

public class RackAndPinion{
    public static final double kGearing  = 1;


    public static final Distance kInDistance = Units.Inches.of(0.0);
    public static final Distance kOutDistance = Units.Inches.of(11.56703288);

    public static final Distance kRadius = Units.Inches.of(34.0 / 20.0).div(2.0); //pitch radius
	public static final DistanceAngleConverter converter = new DistanceAngleConverter(kRadius);

    public static final Angle kEpsilonThreshold = converter.toAngle(Units.Inches.of(0.5));//Amount of error the motor is willing to accept

    //Need these for the servo motor base class
    public static final Angle kInAngle = converter.toAngle(kInDistance);
    public static final Angle kOutAngle = converter.toAngle(kOutDistance);



    public static final Current kCurrentThreshold = Amps.of(50); 

    public static final TalonFXConfiguration getTalonFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 1;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

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
            return new MotorIOTalonFXSim(getIOConfig(), new LinearSim(getSimConstants()));
        }
    }


    public static LinearSimConstants getSimConstants(){
        LinearSimConstants constants = new LinearSimConstants();
        constants.gearing = kGearing;
        constants.minHeight = kInDistance;
        constants.maxHeight = kOutDistance;
        constants.simGravity = false;
        constants.startingHeight = kInDistance;
        constants.motor = DCMotor.getKrakenX60Foc(1);
        constants.carriageMass = Units.Pounds.of(8);
        constants.converter = converter;
        
		return constants;
    }

}
    
}
