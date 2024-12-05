// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class PhotonVision {
    PhotonCamera limelight;
    private boolean hasTarget;
    PhotonTrackedTarget target;
    public PhotonVision(){
       limelight = new PhotonCamera("limelight2");
       var result = limelight.getLatestResult();
        hasTarget = result.hasTargets();
        target = result.getBestTarget();
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();
        Transform3d pose = target.getAlternateCameraToTarget();
    }
     

    public PhotonPipelineResult  getPipelineResults(){
        return limelight.getLatestResult();
    }
    public boolean hasTarget(){
        return hasTarget;
    }

    
}

