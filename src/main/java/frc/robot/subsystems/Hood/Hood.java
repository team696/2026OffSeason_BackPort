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
  import frc.lib.io.MotorIO.Setpoint;

  public class Hood extends ServoMotorSubsystem<MotorIOTalonFX> {

      private static Hood hood = null;
      public static synchronized Hood get(){
        if(hood == null){
          hood = new Hood();
        }
        return hood;
      }


      // Motion Magic so every move follows the speed/accel limits in HoodConstants instead of slamming
      public static final Setpoint STOW = Setpoint.withPositionSetpoint(HoodConstants.kStowAngle,2);
      public static final Setpoint MAX_ANGLE = Setpoint.withPositionSetpoint(HoodConstants.kMaxAngle,2);
      public static final Setpoint ZERO = Setpoint.withPositionSetpoint(HoodConstants.Zero,2);
      public static Setpoint TEST_ANGLE = Setpoint.withPositionSetpoint(Units.Degrees.of(HoodConstants.testAngle.getAsDouble()));


    /** Creates a new Hood. */
    private Hood() {
      super(HoodConstants.getMotorIO(), "Hood", HoodConstants.kEpsilonThreshold, HoodConstants.getServoConfig());
      // Hood rests on its bottom hard stop (home, 0 deg) at boot
      setCurrentPosition(HoodConstants.kMinAngle);

    }

    /** Drives the hood into its bottom hard stop (0 deg), zeroes the encoder there, then goes to STOW. Skipped once zeroed. */
    public Command zeroCommand() {
      return zeroIfNeededCommand(HoodConstants.getZeroConfig(), STOW);
    }

    public Command getTestShotCommand() {
      return followSetpointCommand(() -> 
          Setpoint.withMotionMagicSetpoint(
              Units.Degrees.of(HoodConstants.testAngle.getAsDouble()),2
          )
      );
  }

    // The lib's homing runs a voltage setpoint from periodic(). A command that re-asks for home every loop
    // (followSetpointCommand) would cancel it before it finishes, so skip those repeats while homing.
    // The lib finishes homing with its own MOTIONMAGIC setpoint at home, so only STOW/ZERO themselves are skipped.S
    @Override
    public void applySetpoint(Setpoint setpoint) {
      boolean homing = getSetpoint().mode.isVoltageControl();
      if (homing && (setpoint == STOW || setpoint == ZERO)) {
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
