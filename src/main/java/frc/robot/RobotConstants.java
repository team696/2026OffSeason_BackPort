package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.CANBus;

import frc.lib.util.FieldLayout.FieldType;
import frc.robot.Log.SEVERITY;
import frc.robot.subsystems.drive.TunerConstants;

public class RobotConstants {
	public static final SEVERITY LOGGING_SEVERITY = SEVERITY.EVERYTHING; 

	public static final CANBus RIOBUS = new CANBus("rio");
	public static final CANBus CANIVORE_BUS = new CANBus("CANivore");

	public static final Transform2d SHOOTER_OFFSET =
			new Transform2d(Inches.of(7), BaseUnits.DistanceUnit.zero(), Rotation2d.kZero); // 7in forward of center, shooting out the front

	public static FieldType currentFieldType = FieldType.Andymark;
	public static boolean isRedAlliance;
	public static boolean isRedAlliance() {
        return isRedAlliance;
    }

	public static class DriveConstants{
        public final static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
	    public final  static double MaxAngularRate = RotationsPerSecond.of(5.).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	
	    public static final double kToleranceDegree = 0.5;
	    public static final double kToleranceSpeed = 0.01;

    }




}
