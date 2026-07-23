// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Currently setup for 1 camera, instructions on adding more than 1 cameras will be stated below
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

import com.ctre.phoenix6.Utils;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.SimulatedCameraIOConfig;
import frc.lib.bases.CameraSubsystem.VisionConfig;
import frc.lib.io.vision.CameraIO;
//import frc.lib.io.vision.limelight.AprilTagLimelightCameraIO;
import frc.lib.io.vision.limelight.CombinedLimelightCameraIO;
import frc.lib.io.vision.sim.AprilTagSimulatedCameraIO;
import frc.lib.util.vision.CameraPipeline;
import frc.lib.util.vision.CameraPipelineType;

/** Add your docs here. */
public class CameraConstants {

    //Standard deviation
    public static final Vector<N3> DEFAULT_STD_DEVIATION = 
        VecBuilder.fill(0.3,0.3,99999999.0); //Basically rejects all rotational values, relying purely on the pigeon

    public static final class FrontConstants{ //Front Camera

        // Necessary for stuff like object detection and simulation 
        public static final Pose3d OFFSET_FROM_CENTER = new Pose3d(
            Inches.of(0.0),   // x — forward/back from center
            Inches.of(0.0),   // y — left/right from center
            Inches.of(0.0),   // z — height from ground
            new Rotation3d(
                Degrees.of(0.0),   // roll
                Degrees.of(0.0),   // pitch — angle camera tilts up/down
                Degrees.of(0.0))); // yaw — angle camera faces left/right


            //Standard Limelight set up, even a monkey can understand
            public static CameraIOConfig getCameraConfig(){
                CameraIOConfig config = new CameraIOConfig();
                config.name = "limelight-front";
                config.robotToCameraOffset = OFFSET_FROM_CENTER;
                config.aprilTagVisionStdDevs = DEFAULT_STD_DEVIATION;
                return config;
            }

            //Same thing here, some things are needed to explain
            public static SimulatedCameraIOConfig getSimConfig(){
                SimulatedCameraIOConfig config = new SimulatedCameraIOConfig();
                config.resolutionHeightPixels = 600; //resoultion size
                config.resolutionWidthPixels = 800;
                config.kErrThreshold = Degrees.of(25.0); //Error treshold
                config.maxUpdateTagDistance = Meters.of(4.0);
                config.kFieldOfView = Degrees.of(63.0);   // realistic Limelight 3 FOV
                config.calibErrorPx = 0.25; // pixel noise
                config.calibErrorPy = 0.25;
                config.latency = Milliseconds.of(20.0);   // realistic processing delay
                config.latencyStdDevs = Milliseconds.of(5.0);
                return config;
            }

            //Checks if its in simulation
            public static CameraIO getIO(){
                if(Utils.isSimulation()){
                    return new AprilTagSimulatedCameraIO(getCameraConfig(), getSimConfig());
                }

                 return new CombinedLimelightCameraIO(getCameraConfig()); 
            }


        }

            public static VisionConfig getConfig(){
                VisionConfig config = new VisionConfig();
                config.name = "Vision";
                config.cameras = new CameraIO[]{FrontConstants.getIO()};
                config.agreedTranslationUpdatesThreshold = 5;
                config.agreedTranslationUpdateEpsilon = Centimeters.of(20.0);
                return config;
            }

        public static class PipelineConstants{
                public static final CameraPipeline APRIL_TAG = 
                new CameraPipeline(CameraPipelineType.APRIL_TAG, "April Tag", 0);
;
                public static final CameraPipeline GAME_PIECE = 
                new CameraPipeline(CameraPipelineType.OBJECT_DETECTION, "Game Piece", 1);;
        }


}
