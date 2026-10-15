// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.regression;


import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class AirtimeRegression {
    //hub
    public static final InterpolatingDoubleTreeMap distanceToTimeToHub = new InterpolatingDoubleTreeMap();

    
    public static void insertPointHub(Distance distance, Time time) {
        distanceToTimeToHub.put(distance.baseUnitMagnitude(), time.baseUnitMagnitude());
    }

    //data for the hub
    static {
        // simulated time of flight, tools/shooter_sim/shooter_sim.py
        insertPointHub(Meter.of(2.25), Second.of(0.731));
        insertPointHub(Meter.of(2.5), Second.of(0.795));
        insertPointHub(Meter.of(3.0), Second.of(0.887));
        insertPointHub(Meter.of(3.5), Second.of(0.940));
        insertPointHub(Meter.of(4.0), Second.of(1.006));
        insertPointHub(Meter.of(4.5), Second.of(1.064));
        insertPointHub(Meter.of(5.0), Second.of(1.114));
        insertPointHub(Meter.of(5.5), Second.of(1.159));
    }

    public static Time getHubShotTime(Distance distance) {
        return Time.ofBaseUnits(distanceToTimeToHub.get(distance.baseUnitMagnitude()), Second);
    }

    //Ferry (neutral)
    public static final InterpolatingDoubleTreeMap distanceToTimeToFerryNetural = new InterpolatingDoubleTreeMap();

    public static void insertPointFerryNeutral(Distance distance, Time time) {
        distanceToTimeToFerryNetural.put(distance.baseUnitMagnitude(), time.baseUnitMagnitude());
    }

    static {
        // simulated time of flight, tools/shooter_sim/shooter_sim.py
        insertPointFerryNeutral(Meter.of(4.5), Second.of(1.322));
        insertPointFerryNeutral(Meter.of(6.0), Second.of(1.350));
        insertPointFerryNeutral(Meter.of(7.5), Second.of(1.346));
    }

    public static Time getFerryAllanceNeutral(Distance distance) {
        return Time.ofBaseUnits(distanceToTimeToFerryNetural.get(distance.baseUnitMagnitude()), Second);
    }





}
