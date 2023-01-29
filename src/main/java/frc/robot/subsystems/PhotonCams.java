package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.ExternalLib.SpectrumLib.drivers.PhotonColorSensors;

public class PhotonCams extends SubsystemBase{
    
    // creates photonVision Camera object 
    private final PhotonCamera visionCam;

    private final PhotonPoseEstimator  estimator;


    public PhotonCams(PhotonCamera camera) throws IOException{
        //sets the camera in the subsytem, as the camera given. 
        this.visionCam = camera;
        // turns off "driver mode"
        visionCam.setDriverMode(false);
        // turns off LEDs
        visionCam.setLED(VisionLEDMode.kOff);

        estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile), PoseStrategy.AVERAGE_BEST_TARGETS, camera, null);
        
    }
    // returns the position of the tag, relative to the robot
    


    // returns detected tag ID
    public int getTagID(){
        var res = visionCam.getLatestResult();
        if(res.hasTargets()){
          return  res.getBestTarget().getFiducialId();
        }else return 0;
    }
    // returns if the camera has a target
    public boolean hasTargets(){
        var res = visionCam.getLatestResult();
        return res.hasTargets();
    }
    



    @Override 
    public void periodic(){
        
    }

}