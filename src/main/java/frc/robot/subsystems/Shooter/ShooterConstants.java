// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */  
public class ShooterConstants {
    public static AngularVelocity TestShotSpeed = RotationsPerSecond.of(SmartDashboard.getNumber("Test Speed", 0));
    public static final AngularVelocity IdleSpeed = RotationsPerSecond.of(0);
    public static final AngularVelocity FerrySpeed = RotationsPerSecond.of(20);
    public static final AngularVelocity ShotSpeed = RotationsPerSecond.of(30);

    public static final AngularVelocity epsilonThreshold = RotationsPerSecond.of(0.1);

    public static final double kGearing = 1;


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

        config_talon.Slot0.kP = 1;
        config_talon.Slot0.kI = 0;
        config_talon.Slot0.kD = 0;

        config_talon.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
        config_talon.CurrentLimits.StatorCurrentLimit = 90.0;

        config_talon.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
        config_talon.CurrentLimits.SupplyCurrentLimit = 45.0;
        config_talon.Feedback.SensorToMechanismRatio = kGearing;
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
