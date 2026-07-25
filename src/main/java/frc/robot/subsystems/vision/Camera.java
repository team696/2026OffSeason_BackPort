// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.bases.CameraSubsystem;
/** Add your docs here. */
public class Camera extends CameraSubsystem{

        public static Camera m_Camera;
        public static synchronized Camera get() {
        if (m_Camera == null)
            m_Camera = new Camera();
        return m_Camera;
    }

    public Camera(){
        super(CameraConstants.getConfig());
    }


    //If set to this, it will use Localization to map its position on the field
    public Command switchToAprilTagPipeline(){
        return runOnce(() -> setPipeline(CameraConstants.PipelineConstants.APRIL_TAG));
    }

    //Once changed to this, it can detect objects based off of the Limelight detection
    public Command switchToNueralNetworkPipeline(){
        return runOnce(()-> setPipeline(CameraConstants.PipelineConstants.GAME_PIECE));
    }

    

}
