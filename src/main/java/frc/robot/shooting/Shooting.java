package frc.robot.shooting;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.shooting.motionComp.MotionComp;
import frc.robot.shooting.shotverifier.ShotVerify;
import frc.robot.shooting.shotverifier.ShotVerify;


public class Shooting {
    

    public static final Shooting mInstance = new Shooting();

    private final MotionComp motionComp = new MotionComp();

    private static ShootingState state = ShootingState.HUB;
    
        private Shooting() {}
    
        /** Call this once per loop from Robot.java, same as Swerve/Cameras. */
    public void periodic() {
         motionComp.rejectPose();
    }
    
    
    public static void setState(ShootingState newState) {
        state = newState;
    }

    public static ShootingState getState() {
        return state;
    }

    public boolean shotVerified() {
        return ShotVerify.verifyShot(motionComp);
    }

    public MotionComp getMotionComp() {
        return motionComp;
    }
}