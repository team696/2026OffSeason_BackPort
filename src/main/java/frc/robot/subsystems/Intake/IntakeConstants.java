// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.LinearSim;
import frc.lib.sim.LinearSim.LinearSimConstants;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.lib.util.Util.DistanceAngleConverter;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class IntakeConstants {

   public class RackAndPinion{
    public static final double kGearing  = 30.0/12.0;


    public static final Distance kInDistance = Units.Inches.of(-0.01);
    public static final Distance kOutDistance = Units.Inches.of(14);

    public static final Distance kRadius = Units.Inches.of(34.0 / 20.0).div(2.0); //pitch radius
	public static final DistanceAngleConverter converter = new DistanceAngleConverter(kRadius);

    public static final Angle kEpsilonThreshold = converter.toAngle(Units.Inches.of(0.001));//Amount of error the motor is willing to accept

    //Need these for the servo motor base class
    public static final Angle kInAngle = converter.toAngle(kInDistance);
    public static final Angle kOutAngle = converter.toAngle(kOutDistance);



    public static final Current kCurrentThreshold = Amps.of(50); 

    public static final TalonFXConfiguration getTalonFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 2;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.Feedback.SensorToMechanismRatio = kGearing;

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
        config.mainID = Ports.INTAKE_RACK_AND_PINION.id;
        config.mainBus = Ports.INTAKE_RACK_AND_PINION.bus;
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


    public class Roller{
        public static final double kGearing = 1;

        public static final Voltage kIdleVoltage = Units.Volts.of(0.0);
        public static final Voltage kIntakeVoltage = Units.Volts.of(3.0);
        public static final Voltage kOuttakeVoltage = Units.Volts.of(-3.0);

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
            // config.followerConfig = getTaloFXConfig();
            // config.followerAlignment = new MotorAlignmentValue[] {MotorAlignmentValue.Opposed};
            // config.followerIDs = new int[] {Ports.INTAKE_ROLLERS_FOLLOWER.id};
            // config.followerBuses = new CANBus[] {Ports.INTAKE_ROLLERS_FOLLOWER.bus};
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
