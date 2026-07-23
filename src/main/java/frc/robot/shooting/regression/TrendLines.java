// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.regression;

import edu.wpi.first.math.Pair;

import frc.lib.math.LinearLineCalculator;
import java.util.List;

/** Add your docs here. */
public class TrendLines {

    public static final LinearLineCalculator HUB_TREND_LINE = LinearLineCalculator.bestFit(List.of(
        Pair.of(0.0,0.0)
    ));

    public static final LinearLineCalculator FERRY_TREND_LINE = LinearLineCalculator.bestFit(List.of(
        Pair.of(0.0,0.0)
    ));

}
