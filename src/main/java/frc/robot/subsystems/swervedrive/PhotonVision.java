// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  
    PhotonCamera limelight;
    private boolean hasTarget;
    PhotonTrackedTarget target;
    ArrayList<Double> pose;
    public PhotonVision(){
       limelight = new PhotonCamera("limelight2");
       var result = limelight.getLatestResult();
        hasTarget = result.hasTargets();
        target = result.getBestTarget();
        double pitch = target.getPitch();
        Transform3d pose = target.getAlternateCameraToTarget();
    }
     

    public PhotonPipelineResult  getPipelineResults(){
        return limelight.getLatestResult();
    }
    public boolean hasTarget(){
        return hasTarget;
    }
    public double getYaw(){
      return target.getYaw();
    }

    



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("hasTarget", hasTarget());
    SmartDashboard.putNumber("VisionYaw",getYaw());
    

  }
}
