package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import frc.ExternalLib.SpectrumLib.drivers.PhotonColorSensors;

public class PhotonCams extends SubsystemBase{
    
    // creates photonVision Camera object 
    private final PhotonCamera visionCam;
    // Tag Id being tested
    private static int targetTag = 3;
    // the position of the camera relative to the the robot's center of rotatation.
    private static Transform2d robotToCam = new Transform2d(new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(5)), new Rotation2d(0.0));
    // position of the goal, relative to the target Tag
    private static Pose2d tagToGoal = new Pose2d(1, 0, new Rotation2d(180));
    // Transform2d object, or vector, used to derive the position that the robot must go to. 
    private Transform2d transform;
    // the location of the tag, relative to the location of the robot.
    private Pose2d tagLocation = new Pose2d();
    //
    public PhotonCams(PhotonCamera camera){
        //sets the camera in the subsytem, as the camera given. 
        this.visionCam = camera;
        // turns off "driver mode"
        visionCam.setDriverMode(false);
        // turns off LEDs
        visionCam.setLED(VisionLEDMode.kOff);


        
    }
    // returns the positoion of the tag, relative to the robot
    public Pose2d getTagLocation(Pose2d robotPose){
        //gets latest data from camera
        var res = visionCam.getLatestResult();
        // checks if the camera has a target
        if(res.hasTargets()){
            var targetOpt = res.getBestTarget().getFiducialId();
            // checks if the the tag is Tag 3
            if(targetOpt == targetTag){
                // calculates location to drive to
                var camToTarget = res.getBestTarget().getBestCameraToTarget();
                // translate 3d vector to 2d vector
                 transform = new Transform2d(
                    camToTarget.getTranslation().toTranslation2d(), 
                    camToTarget.getRotation().toRotation2d());
                    // translates robot position to camera position
                var cameraPose = robotPose.transformBy(robotToCam.inverse());
                
                //calculates target position
                Pose2d TargetPose = cameraPose.transformBy(transform);
                
                // sets internal location to the calculated location which is 1m in front of the tag position
                tagLocation = TargetPose.relativeTo(tagToGoal);
                
                // spits out target location, which is 1m in front of the tag position
                return TargetPose.relativeTo(tagToGoal);
               

            }else // if the tag is not the correct one, then spit out the last known location
            return tagLocation;
            
        } 
        else // if no tag is detected, spit out last known tag location
    return tagLocation;
    }

    // spits out last known tag location
    public Pose2d getLastTagLocation(){
        return tagLocation;
        
    }
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