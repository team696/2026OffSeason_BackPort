// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FeedRollers;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.system.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.sim.RollerSim;
import frc.lib.sim.RollerSim.RollerSimConstants;
import frc.robot.Ports;
import frc.robot.Robot;

/** Add your docs here. */
public class FeedRollersConstants {

    public static AngularVelocity IdleSpeed = RotationsPerSecond.of(0);
    public static AngularVelocity RunningSpeed = RotationsPerSecond.of(20);
    public static double kGearing;

    public static MotorIOTalonFXConfig getConfig() {
        MotorIOTalonFXConfig config = new MotorIOTalonFXConfig();

        config.mainID = Ports.FEEDER_ROLLERS.id;
        config.mainBus = Ports.FEEDER_ROLLERS.bus;

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
        return config;
    }

    public static RollerSimConstants getSimConstants() {
        RollerSimConstants consts = new RollerSimConstants();

        consts.motor = DCMotor.getKrakenX60(1);
        consts.gearing = kGearing;
        consts.momentOfInertia = 0.001618297;
        return consts;
    }

    public static MotorIOTalonFX getMotorIO() {
        if (Robot.isReal()) {
            return new MotorIOTalonFX(getConfig());
        } else {
            return new MotorIOTalonFXSim(getConfig(), new RollerSim(getSimConstants()));
        }
    }


}
