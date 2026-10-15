// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.lib.util.tunablenumbers.TunableNumber;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */  
public class ShooterConstants {
    public static final TunableNumber testShotSpeed = new TunableNumber("Test Speed", 0.0);
    public static DoubleSupplier testShotSpeedSupplier = () -> SmartDashboard.getNumber("Test Speed", 0.0);
    public static final AngularVelocity IdleSpeed = RotationsPerSecond.of(0);
    public static final AngularVelocity FerrySpeed = RotationsPerSecond.of(20);
    public static final AngularVelocity ShotSpeed = RotationsPerSecond.of(30);

    public static final AngularVelocity epsilonThreshold = RotationsPerSecond.of(0.1);
    // feed rollers wait until the drum is within this of its target speed
    public static final AngularVelocity kFeedTolerance = RotationsPerSecond.of(2.0);

    public static final double kGearing = 45.0/22.0;


    public static RollerSimConstants getSimConstants() {
        RollerSimConstants consts = new RollerSimConstants();

        consts.gearing = kGearing;
        consts.momentOfInertia = 0.0012143;
        consts.motor = DCMotor.getKrakenX60(2);
        return consts;
    }

    public static MotorIOTalonFXConfig getConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();

        config.mainID = Ports.SHOOTER_MAIN.id;
        config.mainBus = Ports.SHOOTER_MAIN.bus;

        config.followerIDs = new int[] { Ports.SHOOTER_FOLLOWER.id };
        config.followerBuses = new CANBus[] { Ports.SHOOTER_FOLLOWER.bus };
        config.followerAlignment = new MotorAlignmentValue[] { MotorAlignmentValue.Opposed };

        config.time = Seconds;
        config.unit = Rotations;

        TalonFXConfiguration config_talon = new TalonFXConfiguration();

        // Volts per rps of error: pushes back when a ball slows the drum. Raise toward 4-5 if it
        // still sags while feeding, lower it if the drum oscillates or whines at speed.
        config_talon.Slot1.kP = 0.1;
        config_talon.Slot1.kI = 0;
        config_talon.Slot1.kD = 0;

        config_talon.Slot1.kV = 0.245;

        config_talon.MotionMagic.MotionMagicCruiseVelocity = 30.0;
        config_talon.MotionMagic.MotionMagicAcceleration = 30.0;
        config_talon.MotionMagic.MotionMagicJerk = 1000.0;


        config_talon.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config_talon.CurrentLimits.StatorCurrentLimit = 200.0;

        config_talon.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        // 45 A (and Phoenix's default drop to 40 A after 1 s) couldn't keep up with balls going through:
        // the drum hit 40 rps then sagged to 32 while feeding. Two motors at 60 A = up to 120 A from the
        // battery during a burst - lower these if the robot browns out while driving and shooting.
        config_talon.CurrentLimits.SupplyCurrentLimit = 160.0;
        config_talon.CurrentLimits.SupplyCurrentLowerLimit = 50.0;
        config_talon.CurrentLimits.SupplyCurrentLowerTime = 2.0;
        config_talon.Feedback.SensorToMechanismRatio = kGearing;

        config_talon.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.mainConfig = config_talon;
        config.followerConfig = config_talon;


        return config;
    }


    public static MotorIOTalonFX getMotorIO() {
        if (Robot.isReal()) {
            return new MotorIOTalonFX(getConfig());
        } else {
            return new MotorIOTalonFXSim(getConfig(), new RollerSim(getSimConstants()));
        }
    }
}
