// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.regression;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Hood.HoodConstants;

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

    // Hub and passing both go through here, so keep every shot inside the hood's shooting range
    public Angle getHoodAngle(Distance input){
        return Degrees.of(MathUtil.clamp(
            hoodAngleMap.get(input.in(Meters)),
            HoodConstants.kMinShootingAngle.in(Degrees),
            HoodConstants.kMaxAngle.in(Degrees)));
    }

    public AngularVelocity getShooterVeloicty(Distance input){
        return RotationsPerSecond.of(shooterVeloictyMap.get(input.in(Meters)));
    }
}


