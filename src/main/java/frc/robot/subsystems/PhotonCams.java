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
    private final PhotonCamera visionCam;
    private static int targetTag = 3;
    private static Transform2d robotToCam = new Transform2d(new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(5)), new Rotation2d(0.0));
    private static Pose2d tagToGoal = new Pose2d(1, 0, new Rotation2d(180));
    private Transform2d transform;
    private Pose2d tagLocation = new Pose2d();
    public PhotonCams(PhotonCamera camera){
        this.visionCam = camera;
        visionCam.setDriverMode(false);
        visionCam.setLED(VisionLEDMode.kOff);


        
    }
    public Pose2d getTagLocation(Pose2d robotPose){
        var res = visionCam.getLatestResult();
        if(res.hasTargets()){
            var targetOpt = res.getBestTarget().getFiducialId();
            if(targetOpt == targetTag){
                var camToTarget = res.getBestTarget().getBestCameraToTarget();
                 transform = new Transform2d(
                    camToTarget.getTranslation().toTranslation2d(), 
                    camToTarget.getRotation().toRotation2d());
                var cameraPose = robotPose.transformBy(robotToCam.inverse());

                Pose2d TargetPose = cameraPose.transformBy(transform);
                tagLocation = TargetPose.relativeTo(tagToGoal);
                
                return TargetPose.relativeTo(tagToGoal);
               

            }else 
            return tagLocation;
            
        } 
        else 
    return tagLocation;
    }
    public Pose2d getLastTagLocation(){
        return tagLocation;
    }
    public int getTagID(){
        var res = visionCam.getLatestResult();
        if(res.hasTargets()){
          return  res.getBestTarget().getFiducialId();
        }else return 0;
    }
    public boolean hasTargets(){
        var res = visionCam.getLatestResult();
        return res.hasTargets();
    }
    



    @Override 
    public void periodic(){
        
    }

}