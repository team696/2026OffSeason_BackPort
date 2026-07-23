package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.BaseUnits;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.util.FieldLayout.FieldType;
import frc.robot.Log.SEVERITY;

public class RobotConstants {
	public static final SEVERITY LOGGING_SEVERITY = SEVERITY.EVERYTHING; 

	public static final CANBus RIOBUS = new CANBus("rio");
	public static final CANBus CANIVORE_BUS = new CANBus("cv");

	public static final CANBus canivore1 = new CANBus("canivore1", "./logs/example.hoot");
	public static final CANBus rio = new CANBus();
	public static final Transform2d SHOOTER_OFFSET =
			new Transform2d(Inches.of(11.28).unaryMinus(), BaseUnits.DistanceUnit.zero(), Rotation2d.kPi);




	public static FieldType currentFieldType = FieldType.Andymark;
	public static boolean isRedAlliance = false;
	public static boolean isRedAlliance() {
        return isRedAlliance;
    }




}
