package frc.robot.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.io.MotorIO.Setpoint;
import frc.robot.shooting.motionComp.MotionComp;

public class ShootingSetpoint {
    


    public static final ShootingSetpoint mInstance = new ShootingSetpoint();

    private final MotionComp motionComp;

    public ShootingSetpoint() {
        this.motionComp = Shooting.mInstance.getMotionComp();
    }

    public Distance getLookAheadDistance() {
        return Meters.of(motionComp.getTarget().getDistance(motionComp.getBaseTranslation()));
    }

    /** Look-ahead distance plus the dashboard Distance Offset; this is what the LUTs are read at. */
    private Distance getLutDistance() {
        Distance distance = getLookAheadDistance().plus(Meters.of(ShotTuning.distanceOffset.getAsDouble()));
        SmartDashboard.putNumber("Shooting/LUT Distance", distance.in(Meters));
        return distance;
    }

    public Setpoint getShooterSetpoint() {
        boolean isHub = Shooting.getState() == ShootingState.HUB;
        double lutRps = Shooting.getState().getRegression().getShooterVeloicty(getLutDistance()).in(RotationsPerSecond);
        double rps = ShotTuning.applyRps(lutRps, isHub);
        SmartDashboard.putNumber("Shooting/Target RPS", rps);
        return Setpoint.withVelocitySetpoint(RotationsPerSecond.of(rps));
    }

    public Setpoint getHoodSetpoint() {
        boolean isHub = Shooting.getState() == ShootingState.HUB;
        double lutHood = Shooting.getState().getRegression().getHoodAngle(getLutDistance()).in(Degrees);
        double hood = ShotTuning.applyHood(lutHood, isHub);
        SmartDashboard.putNumber("Shooting/Target Hood", hood);
        // slot 2 has the hood gains; the default slot 0 had none, so the hood never moved while shooting
        return Setpoint.withMotionMagicSetpoint(Degrees.of(hood), 2);
    }
}