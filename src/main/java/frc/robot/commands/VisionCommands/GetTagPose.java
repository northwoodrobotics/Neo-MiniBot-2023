package frc.robot.commands.VisionCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonCams;

public class GetTagPose extends CommandBase{
    private final PhotonCams m_cameras; 
    private Timer posetimer;
    
    public GetTagPose(PhotonCams cameras){
        this.m_cameras = cameras;
        posetimer = new Timer();
     
    }

    @Override
    public void initialize(){
        posetimer.reset();
        posetimer.start();
       
       SmartDashboard.putNumber("PreVisonPoseX", m_cameras.getLastTagLocation().getX());
       SmartDashboard.putNumber("PreVisonPoseY", m_cameras.getLastTagLocation().getY());
       SmartDashboard.putNumber("PreVisonPoseRot", m_cameras.getLastTagLocation().getRotation().getDegrees());
    }
    public void execute(){
        m_cameras.getTagLocation(RobotContainer.m_SwerveSubsystem.dt.getPose());
    }
    public void end(boolean interrupted){
         
       SmartDashboard.putNumber("PostVisionPoseX", m_cameras.getLastTagLocation().getX());
       SmartDashboard.putNumber("PostVisonPoseY", m_cameras.getLastTagLocation().getY());
       SmartDashboard.putNumber ("PostVisonPoseRot", m_cameras.getLastTagLocation().getRotation().getDegrees());
    }
       @Override
    public boolean isFinished(){
        return posetimer.hasElapsed(.25);
    }

    

}
