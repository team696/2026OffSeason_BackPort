// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting.regression;


import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.DistanceUnit;
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
        insertPointHub(Meter.of(1), Second.of(1));
        //add more later
    }

    public static Time getHubShotTime(Distance distance) {
        return Time.ofBaseUnits(distanceToTimeToHub.get(distance.baseUnitMagnitude()), Second);
    }

    //ferry (allance)
    public static final InterpolatingDoubleTreeMap distanceToTimeToFerryAllance = new InterpolatingDoubleTreeMap();
    
    public static void insertPointFerryAllance(Distance distance, Time time) {
        distanceToTimeToFerryAllance.put(distance.baseUnitMagnitude(), time.baseUnitMagnitude());
    }

    static {
        insertPointFerryAllance(Meter.of(1), Second.of(1));
        //add more later
    }

    public static Time getFerryAllanceTime(Distance distance) {
        return Time.ofBaseUnits(distanceToTimeToFerryAllance.get(distance.baseUnitMagnitude()), Second);
    }

    //Ferry (neutral)
    public static final InterpolatingDoubleTreeMap distanceToTimeToFerryNetural = new InterpolatingDoubleTreeMap();

    public static void insertPointFerryNeutral(Distance distance, Time time) {
        distanceToTimeToFerryNetural.put(distance.baseUnitMagnitude(), time.baseUnitMagnitude());
    }

    static {
        insertPointFerryNeutral(Meter.of(1), Second.of(1));
        //add more later
    }

    public static Time getFerryAllanceNeutral(Distance distance) {
        return Time.ofBaseUnits(distanceToTimeToFerryNetural.get(distance.baseUnitMagnitude()), Second);
    }





}
