// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.regression;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ShootingRegression {
    private final InterpolatingDoubleTreeMap hoodAngleMap, shooterVeloictyMap;

    public ShootingRegression(
        InterpolatingDoubleTreeMap hoodAngleMap,
        InterpolatingDoubleTreeMap shooterVelocityMap
    )
    {
        this.hoodAngleMap = hoodAngleMap;
        this.shooterVeloictyMap = shooterVelocityMap;
    }

    public Angle getHoodAngle(Distance input){
        return Degrees.of(hoodAngleMap.get(input.in(Meters)));
    }

    public AngularVelocity getShooterVeloicty(Distance input){
        return RotationsPerSecond.of(shooterVeloictyMap.get(input.in(Meters)));
    }
}


