// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.util.FieldLayout;
import frc.robot.shooting.regression.LookUpTables.ScoringHoodLUT;
import frc.robot.shooting.regression.LookUpTables.ScoringVelocityLUT;
import frc.robot.shooting.regression.LookUpTables.NeturalZoneHoodLUT;
import frc.robot.shooting.regression.LookUpTables.NetrualZoneVelocityLUT;
import frc.robot.shooting.regression.ShootingRegression;

public enum ShootingState {
    HUB(
        () -> FieldLayout.kBlueHub,
        new ShootingRegression(ScoringHoodLUT.scoringhoodLUT, ScoringVelocityLUT.scoringvelocityLUT)
    ),
    NEUTRAL_FERRY(
        () -> FieldLayout.kBlueLeftNeutralFerry, // pick whichever ferry target applies
        new ShootingRegression(NeturalZoneHoodLUT.neturalzonehoodLUT, NetrualZoneVelocityLUT.neturalzonevelocityLUT)
    );

    private final java.util.function.Supplier<Translation2d> blueTarget;
    private final ShootingRegression regression;

    ShootingState(java.util.function.Supplier<Translation2d> blueTarget, ShootingRegression regression) {
        this.blueTarget = blueTarget;
        this.regression = regression;
    }

    public Translation2d getTarget(boolean isRedAlliance) {
        return FieldLayout.handleAllianceFlip(blueTarget.get(), isRedAlliance);
    }

    public ShootingRegression getRegression() {
        return regression;
    }
}