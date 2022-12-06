package frc.robot.commands.ActionCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonCams;
import frc.swervelib.SwerveSubsystem;

public class DriveToTag extends CommandBase{
    private final SwerveSubsystem m_Swerve;
    private final PhotonCams m_Cameras;
    private PathPlannerTrajectory Route2Tag;
    private Pose2d TagPose;
    private Transform2d robotToTag;
    

    public DriveToTag(SwerveSubsystem m_SwerveSubsystem, PhotonCams camera){
        this.m_Swerve = m_SwerveSubsystem;
        this.m_Cameras = camera;
    }
    @Override
    public void initialize(){
        TagPose = m_Cameras.getTagLocation(m_Swerve.dt.getPose());
        robotToTag = new Transform2d(m_Swerve.dt.getPose(), TagPose);


        Route2Tag = PathPlanner.generatePath(
            new PathConstraints(2, 1), 
            new PathPoint(m_Swerve.dt.getPose().getTranslation(),robotToTag.getRotation(),m_Swerve.dt.getGyroscopeRotation() ), 
            new PathPoint(TagPose.getTranslation(), robotToTag.getRotation(), TagPose.getRotation())
            );
        
    }

    @Override
    public void execute(){
        m_Swerve.dt.createCommandForTrajectory(Route2Tag, m_Swerve);
    }
    @Override
    public void end(boolean interrupted){
        
    }


}
    

