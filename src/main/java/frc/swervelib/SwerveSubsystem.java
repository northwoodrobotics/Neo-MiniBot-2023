// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervelib;

import frc.wpiClasses.QuadSwerveSim;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModuleState[] states;
  private SwerveModulePosition[] positions;
  private swerveModuleIOInputsAutoLogged[] inputs = new swerveModuleIOInputsAutoLogged[]{
    new swerveModuleIOInputsAutoLogged(), 
    new swerveModuleIOInputsAutoLogged(), 
    new swerveModuleIOInputsAutoLogged(), 
    new swerveModuleIOInputsAutoLogged(),
  };
  
 
  private ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
  public SwerveDrivetrainModel dt;

  public SwerveSubsystem(SwerveDrivetrainModel dt) {
    this.dt = dt;
    modules = dt.getRealModules();
  }  

  @Override
  public void periodic() {

    states = dt.getSwerveModuleStates();
    dt.setModulePositions();
    positions = dt.getModulePositions();

    

    if (states != null) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_FWD_REV_SPEED_MPS);

      modules.get(0).set(states[0].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[0].angle.getRadians());
      modules.get(1).set(states[1].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[1].angle.getRadians());
      modules.get(2).set(states[2].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[2].angle.getRadians());
      modules.get(3).set(states[3].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[3].angle.getRadians());


      dt.m_poseEstimator.update(dt.getGyroscopeRotation(), 

      new SwerveModulePosition[]{
        positions[0],
        positions[1],
        positions[2],
        positions[3]  
      }
      
      
      );
      
      dt.m_tracker.update(dt.getGyroscopeRotation(), dt.gyro.getAccelerlationArray());
      
      for (int i = 0; i<4; i++){
        modules.get(i).updateInputs(inputs[i]);
        Logger.getInstance().processInputs("DriveModule"+Integer.toString(i), inputs[i]);
      }

    }

    dt.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    dt.update(DriverStation.isDisabled(), 13.2);
  }
}
