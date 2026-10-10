package frc.robot.shooting;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Time;
import frc.lib.util.FieldLayout;
import frc.robot.shooting.regression.AirtimeRegression;
import frc.robot.shooting.regression.LookUpTables.ScoringHoodLUT;
import frc.robot.shooting.regression.LookUpTables.ScoringVelocityLUT;
import frc.robot.shooting.regression.LookUpTables.NeturalZoneHoodLUT;
import frc.robot.shooting.regression.LookUpTables.NetrualZoneVelocityLUT;
import frc.robot.shooting.regression.ShootingRegression;

public enum ShootingState {
    HUB(
        () -> FieldLayout.kBlueHub,
        new ShootingRegression(ScoringHoodLUT.scoringhoodLUT, ScoringVelocityLUT.scoringvelocityLUT),
        AirtimeRegression.distanceToTimeToHub, 
        Seconds.of(0.0) 
    ),
    NEUTRAL_FERRY_LEFT(
        () -> FieldLayout.kBlueLeftNeutralFerry, 
        new ShootingRegression(NeturalZoneHoodLUT.neturalzonehoodLUT, NetrualZoneVelocityLUT.neturalzonevelocityLUT),
        AirtimeRegression.distanceToTimeToFerryNetural, // TODO: Change this to your Ferry airtime map
        Seconds.of(0.0) 
    ),
    NEUTRAL_FERRY_RIGHT(
        () -> FieldLayout.kBlueRightNeutralFerry, 
        new ShootingRegression(NeturalZoneHoodLUT.neturalzonehoodLUT, NetrualZoneVelocityLUT.neturalzonevelocityLUT),
        AirtimeRegression.distanceToTimeToFerryNetural, // TODO: Change this to your Ferry airtime map
        Seconds.of(0.0) 
    );

    private final java.util.function.Supplier<Translation2d> blueTarget;
    private final ShootingRegression regression;
    private final InterpolatingDoubleTreeMap distanceToAirTimeMap; 
    private final Time lookAheadTimeOffset;

    ShootingState(
            java.util.function.Supplier<Translation2d> blueTarget, 
            ShootingRegression regression,
            InterpolatingDoubleTreeMap distanceToAirTimeMap,
            Time lookAheadTimeOffset) {
        this.blueTarget = blueTarget;
        this.regression = regression;
        this.distanceToAirTimeMap = distanceToAirTimeMap;
        this.lookAheadTimeOffset = lookAheadTimeOffset;
    }
    
    public Translation2d getLocationTranslation(boolean isRedAlliance) {
        return FieldLayout.handleAllianceFlip(blueTarget.get(), isRedAlliance);
    }

    public ShootingRegression getRegression() {
        return regression;
    }

    // Added to satisfy ShotVisualizer
    public InterpolatingDoubleTreeMap getDistanceToFuelAirTimeMap() {
        return distanceToAirTimeMap;
    }

    // Added to satisfy ShotVisualizer
    public Time getLookAheadTimeOffset() {
        return lookAheadTimeOffset;
    }
}