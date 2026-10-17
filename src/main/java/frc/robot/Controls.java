// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/*
 *  Define Controls as seen on control, DO NOT NAME IT WHAT IT ACTUALLY DOES
 *
 *  All Joysticks Should have Positive be forward
 */
public final class Controls {

	public class xboxController{
		public static CommandXboxController controller = new CommandXboxController(0);
		
		public static final DoubleSupplier leftJoyY = () -> controller.getRawAxis(1);
		public static final DoubleSupplier leftJoyX = () -> controller.getRawAxis(0);
		public static final DoubleSupplier rightJoyX = () -> controller.getRawAxis(4);

		public static final Trigger A = controller.a();
		public static final Trigger B = controller.b();
		public static final Trigger X = controller.x();
		public static final Trigger Y = controller.y();

		public static final Trigger RB = controller.rightBumper();
		public static final Trigger LB = controller.leftBumper();

		public static final Trigger LT = controller.leftTrigger(0.5);
		public static final Trigger RT = controller.rightTrigger(0.5);

	}

}
