  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.subsystems.Hood;

  import static edu.wpi.first.units.Units.Degrees;

  import edu.wpi.first.units.Units;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj2.command.Command;
  import frc.lib.bases.ServoMotorSubsystem;
  import frc.lib.io.MotorIOTalonFX;
  import frc.lib.io.MotorIO.Mode;
  import frc.lib.io.MotorIO.Setpoint;
  import frc.lib.util.Util;

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
      public static Setpoint TEST_ANGLE = Setpoint.withPositionSetpoint(Units.Degrees.of(HoodConstants.testAngle.getAsDouble()));


    /** Creates a new Hood. */
    private Hood() {
      super(HoodConstants.getMotorIO(), "Hood", HoodConstants.kEpsilonThreshold, HoodConstants.getServoConfig());
      setCurrentPosition(HoodConstants.kMinAngle);

    }

    public Command getTestShotCommand() {
      return followSetpointCommand(() -> 
          Setpoint.withPositionSetpoint(
              Units.Degrees.of(HoodConstants.testAngle.getAsDouble())
          )
      );
  }

    // The lib's homing runs a voltage setpoint from periodic(). A command that re-asks for home every loop
    // (followSetpointCommand) would cancel it before it finishes, so skip those repeats while homing.
    // The lib finishes homing with a MOTIONMAGIC setpoint, which still goes through.
    @Override
    public void applySetpoint(Setpoint setpoint) {
      boolean homing = getSetpoint().mode.isVoltageControl();
      boolean positionAtHome = setpoint.mode == Mode.POSITIONPID
          && Util.epsilonEquals(setpoint.baseUnits, HoodConstants.kMinAngle.baseUnitMagnitude(), epsilonThreshold.baseUnitMagnitude());
      if (homing && positionAtHome) {
        return;
      }
      super.applySetpoint(setpoint);
    }

        @Override
      public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Hood Angle", this.getPosition().in(Degrees));
      }


  }
