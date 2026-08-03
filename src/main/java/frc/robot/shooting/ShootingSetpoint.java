package frc.robot.shooting;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.shooting.motionComp.MotionComp;

public class ShootingSetpoint {
    


    public static final ShootingSetpoint mInstance = new ShootingSetpoint();

    private final MotionComp motionComp;

    public ShootingSetpoint() {
        this.motionComp = Shooting.mInstance.getMotionComp();
    }

    private Distance getLookAheadDistance() {
        return Meters.of(motionComp.getTarget().getDistance(motionComp.getBaseTranslation()));
    }

    public Setpoint getShooterSetpoint() {
        return Setpoint.withVelocitySetpoint(Shooting.getState().getRegression().getShooterVeloicty(getLookAheadDistance()));
    }

    public Setpoint getHoodSetpoint() {
        return Setpoint.withPositionSetpoint(Shooting.getState().getRegression().getHoodAngle(getLookAheadDistance()));
    }
}