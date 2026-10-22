// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FeedRollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.bases.FlywheelMotorSubsystem;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIO.Setpoint;

public class FeedRollers extends FlywheelMotorSubsystem<MotorIOTalonFX> {
  /** Creates a new FeedRollers. */

    private static FeedRollers roller = null;
    public static synchronized FeedRollers get(){
      if(roller == null){
        roller = new FeedRollers();
      }
      return roller;
    }

  public static Setpoint IDLE = Setpoint.COAST;
  public static Setpoint RUNNING = Setpoint.withVelocitySetpoint(FeedRollersConstants.RunningSpeed, 1);
  // Returns a new Command every call, a single shared Command can't be used in more than one group
    // public static Command running() {
    //   return get().followSetpointCommand(() -> {
    //     AngularVelocity vel = Shooting.getState().getRegression().getShooterVeloicty(ShootingSetpoint.mInstance.getLookAheadDistance()).times(FeedRollersConstants.PercentOfLUT).times(-1);
    //     return Setpoint.withVelocitySetpoint(vel, 1);
    //   });
    // }
  
    private FeedRollers() {
    super(
      FeedRollersConstants.getMotorIO(),
      "FeedRollers",
      FeedRollersConstants.epsilonThreshold,
      FeedRollersConstants.debounder
    );

    applySetpoint(IDLE);
  }

  public Command getTestSpeedCommand() {
    return followSetpointCommand(() ->
        Setpoint.withVelocitySetpoint(
            RotationsPerSecond.of(FeedRollersConstants.testSpeed.getAsDouble()), 1
        )
    );
  }


  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("FeedRollers Velocity", this.getVelocity().in(RotationsPerSecond));
  }
}
