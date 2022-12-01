package frc.ExternalLib.NorthwoodLib.NorthwoodDrivers;


import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.WPIUtilJNI;

public class GyroTracker {
    private Pose2d m_PoseMeters; 
    private double m_prevTimeSeconds = -1; 

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;
    
    GyroTracker(short[]accelArray, Rotation2d gyroAngle, Pose2d InitalPose){
          

    }
    


}
