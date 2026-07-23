package frc.robot.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FieldLayout;

public class Shooting {
    public static class ShootingState {
        public Translation2d getLocationTranslation(boolean isRedAlliance) {
            return FieldLayout.kBlueHub;
        }

        public InterpolatingDoubleTreeMap getDistanceToFuelAirTimeMap() {
            return new InterpolatingDoubleTreeMap();
        }

        public Time getLookAheadTimeOffset() {
            return edu.wpi.first.units.Units.Seconds.zero();
        }
    }

    private static final ShootingState mState = new ShootingState();

    public static ShootingState getState() {
        return mState;
    }
}
