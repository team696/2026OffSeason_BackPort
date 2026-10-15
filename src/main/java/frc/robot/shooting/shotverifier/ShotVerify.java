// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.shotverifier;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.shooting.motionComp.MotionComp;
import frc.robot.subsystems.drive.Swerve;

/** Add your docs here. */
public class ShotVerify {

    public static final Angle ACCEPTABLE_TILT = Degrees.of(5);
    public static final Angle SHOT_ALIGNMENT_TOLERANCE = Degrees.of(7.0);

    public static boolean isTiled(){ //Very useful if we are beached
        var pigeon = Swerve.get().getPigeon2();
        if(!pigeon.isConnected()){return false;} // can't read tilt without the pigeon, don't block shots

        Angle pitch = pigeon.getPitch().getValue();
        Angle roll = pigeon.getRoll().getValue();
    
        return pitch.abs(Degrees) > ACCEPTABLE_TILT.in(Degrees) || roll.abs(Degrees) > ACCEPTABLE_TILT.in(Degrees);
    }

    public static boolean isAligned(MotionComp motioncomp){
        Angle error = motioncomp.getDriveError(motioncomp.getTarget());
        return error.abs(Degrees) <= SHOT_ALIGNMENT_TOLERANCE.in(Degrees);
    }

    public static boolean verifyShot(MotionComp motioncomp){
        boolean titled = isTiled();
        boolean aligned = isAligned(motioncomp);
        return !titled && aligned;
    }

}
