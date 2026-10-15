// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.PivotSim.PivotSimConstants;
import frc.lib.util.tunablenumbers.TunableNumber;
import frc.lib.sim.PivotSimHelper;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class HoodConstants {
    // 12T motor pulley -> 24T pulley, 10T (10 DP) pinion on that shaft -> 180T hood sector = 36:1
    public static final double kGearing = (24.0 / 12.0) * (180.0 / 10.0);

    // 0 = home (bottom hard stop), ~33 deg measured max travel. Also the max for passing shots.
    public static final Angle kMaxAngle = Units.Degrees.of(33.0);
    public static final Angle kMinAngle = Units.Degrees.of(0.0);
    // Lowest angle used for shooting: the old "48 deg" minimum was in 2:1 units, 48 / 18 = 2.67 real deg
    public static final Angle kMinShootingAngle = Units.Degrees.of(48.0 / 18.0);
    public static final Angle Zero = Units.Degrees.of(0);
    public static final TunableNumber testAngle = new TunableNumber("Test Angle", 0.0);
    public static DoubleSupplier testAngleSupplier = () -> SmartDashboard.getNumber("Test Angle", HoodConstants.kMinAngle.in(Units.Degrees));

    public static final Angle kEpsilonThreshold = Units.Degrees.of(0.5);
    
    //Add test angles

    //BS from 1678
    public static final Voltage kSimStaticFrictionVoltage = Units.Volts.of(0.0);
	public static final Voltage kSimKineticFrictionVoltage = Units.Volts.of(0.0);
	public static final AngularVelocity kSimStictionVelocity = Units.DegreesPerSecond.of(2.0);
	public static final double kSimViscousFrictionVoltsPerRadPerSec = 0.38;

    public static final TalonFXConfiguration getTalonFXConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Gains are per mechanism rotation, so they scale with kGearing; these are the old
        // values (kP 7, kV 0.24 at 2:1) multiplied by 18 to keep the same motor-side behavior.
        config.Slot2.kP = 126;
        config.Slot2.kI = 0;
        config.Slot2.kD = 0;

        config.Slot2.kV = 4.32;

        // The lib finishes homing with a slot 0 Motion Magic setpoint, so slot 0 needs the same gains
        config.Slot0.kP = config.Slot2.kP;
        config.Slot0.kI = config.Slot2.kI;
        config.Slot0.kD = config.Slot2.kD;
        config.Slot0.kV = config.Slot2.kV;

        config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.StatorCurrentLimit = 40.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config.CurrentLimits.SupplyCurrentLimit = 30.0;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxAngle.in(Units.Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinAngle.in(Units.Rotations);

        config.Feedback.SensorToMechanismRatio = kGearing;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        return config;
    }

    public static MotorIOTalonFXConfig getIOConfig(){
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();

        config.mainConfig = getTalonFXConfig();
        config.mainID = Ports.HOOD.id;
        config.mainBus = Ports.HOOD.bus;
        config.unit = Units.Degrees;
        config.time = Units.Seconds;

        return config;
    }

    public static PivotSimConstants getSimConstants(){
        PivotSimConstants constants = new PivotSimConstants();
        constants.motor = DCMotor.getKrakenX44(1);
        constants.armLength = Units.Meters.of(0.5);
        constants.gearing = kGearing;
        constants.momentOfInertia = Units.KilogramSquareMeters.of(0.0614835912);
        constants.mechanismStartPos = kMinAngle;
		constants.mechanismMaxHardStop = kMaxAngle;
		constants.mechanismMinHardStop = kMinAngle;

        return constants;
    }

    public static MotorIOTalonFX getMotorIO(){
        if(Robot.isReal()){
            return new MotorIOTalonFX(getIOConfig());
        }
        else{
            return new MotorIOTalonFXSim(getIOConfig(), 
                        new PivotSimHelper(
							getSimConstants(),
							kSimStaticFrictionVoltage,
							kSimKineticFrictionVoltage,
							kSimStictionVelocity,
							kSimViscousFrictionVoltsPerRadPerSec));
        }
    }

    public static ServoHomingConfig getServoConfig() {
		ServoHomingConfig servoConfig = new ServoHomingConfig();
		servoConfig.kHomingTimeout = Units.Seconds.of(0.5);
		servoConfig.kHomingVoltage = Units.Volts.of(-1.5); 
		servoConfig.kSetHomedVelocity = Units.DegreesPerSecond.of(5.0);
		servoConfig.kHomePosition = kMinAngle;
		return servoConfig;
	}


}
