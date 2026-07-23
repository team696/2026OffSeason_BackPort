// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Necessary to bridge our already existing Swerve Code with 1678's library
//Relatively simple, adds additional methods to bridge between the two unrelated classes. 

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisVelocities;
import edu.wpi.first.math.linalg.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  public static final Drive mInstance = new Drive();

  public Drive() { }

  //Self explanitory
  public Pose2d getPose(){
    return Swerve.get().getState().Pose;
  }

  public DriveVelocityState getState() {
    return new DriveVelocityState(Swerve.get().getState().Velocity);
  }

  //Again, self explanitory
  public static class DriveVelocityState {
    public final ChassisVelocities Speeds;
    public DriveVelocityState(ChassisVelocities speeds) { this.Speeds = speeds; }
  }

  //Add Vision Update through pose, the current timestamp, and standard deviations
  public void addVisionUpdate(Pose2d pose, Time timestamp, Matrix<N3 , N1> stdDevs){
    Swerve.get().addVisionUpdate(pose, timestamp, stdDevs);
  }


  //Refer to the method in the swerve class for the mathmatical reference
  public Pose2d getLookAheadPoseWithoutHeading(Time dt){
    return Swerve.get().getLookAheadPoseWithoutHeading(dt);
  }

  //Refer to the method in the swerve class for explination
  public boolean pitchStable() {
    return Swerve.get().pitchStable();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
