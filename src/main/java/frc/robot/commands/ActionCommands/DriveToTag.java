package frc.robot.commands.ActionCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonCams;
import frc.swervelib.SwerveSubsystem;

public class DriveToTag extends CommandBase{
    private final SwerveSubsystem m_Swerve;
    private final PhotonCams m_Cameras;
    private PathPlannerTrajectory Route2Tag;
    private Pose2d TagPose;
    private Transform2d robotToTag;
    private Timer posetimer;
    
    

    public DriveToTag(SwerveSubsystem m_SwerveSubsystem, PhotonCams camera){
        this.m_Swerve = m_SwerveSubsystem;
        this.m_Cameras = camera;
        posetimer = new Timer();
    }
    @Override
    public void initialize(){
        TagPose = m_Cameras.getTagLocation(m_Swerve.dt.getPose());
        robotToTag = new Transform2d(m_Swerve.dt.getPose(), TagPose);


        Route2Tag = PathPlanner.generatePath(
            new PathConstraints(6, 4), 
            new PathPoint(m_Swerve.dt.getPose().getTranslation(),robotToTag.getRotation(),m_Swerve.dt.getGyroscopeRotation() ), 
            new PathPoint(TagPose.getTranslation(), robotToTag.getRotation(), TagPose.getRotation())
            );
        
    }

    @Override
    public void execute(){
        RobotContainer.TargetTrajectory = Route2Tag;
      
    }
    @Override 
    public boolean isFinished(){
     return    posetimer.hasElapsed(0.25);
    }
    @Override
    public void end(boolean interrupted){
        
    }


}
    

