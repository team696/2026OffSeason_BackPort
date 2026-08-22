// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class Hood extends ServoMotorSubsystem<MotorIOTalonFX> {

    private static Hood hood = null;
    public static synchronized Hood get(){
      if(hood == null){
        hood = new Hood();
      }
      return hood;
    }


    public static final Setpoint STOW = Setpoint.withPositionSetpoint(HoodConstants.kMinAngle);
    public static final Setpoint MAX_ANGLE = Setpoint.withPositionSetpoint(HoodConstants.kMaxAngle);
    public static Setpoint TEST_ANGLE = Setpoint.withPositionSetpoint(HoodConstants.TestAngle);


  /** Creates a new Hood. */
  private Hood() {
    super(HoodConstants.getMotorIO(), "Hood", HoodConstants.kEpsilonThreshold, HoodConstants.getServoConfig());
    setCurrentPosition(HoodConstants.kMinAngle);

  }

  // Inside Hood.java
  public Command getTestShotCommand() {
    return followSetpointCommand(() -> 
        Setpoint.withPositionSetpoint(
            Units.Degrees.of(SmartDashboard.getNumber("Test Angle", 0))
        )
    );
}

      @Override
    public void periodic() {
      super.periodic();
      SmartDashboard.putNumber("Hood Angle", this.getPosition().in(Rotations));
    }


}
