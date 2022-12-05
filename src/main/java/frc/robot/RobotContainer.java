// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.ExternalLib.SpectrumLib.gamepads.SpectrumXbox;
import frc.ExternalLib.SpectrumLib.gamepads.mapping.ExpCurve;
import frc.robot.commands.DriveCommands.CalibrateGyro;
import frc.robot.commands.DriveCommands.TeleopDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SwerveDrivetrainModel dt;
  public static SwerveSubsystem m_SwerveSubsystem;
  /**
   * SpectrumXbox(0, 0.1, 0.1); is an xbox controller with baked in buttons,
   * triggers, and other logic already there.
   * within the object parameters, the first one is the port, the second and third
   * parameters are deadzone sizes.
   **/
  public static SpectrumXbox driver = new SpectrumXbox(0, 0.1, 0.2);
  private static ShuffleboardTab master = Shuffleboard.getTab("master");

  /**
   * Exp Curves are exponential curves, think parabolas. the first value controls
   * how agressive the curve is, (mess with this one first)
   * the second is it's vertical offset from zero
   * the third value is how stretched or squeezed it is.
   * The final value is how large the deadzone in the middle will be.
   * 
   */
  public static ExpCurve throttleCurve = new ExpCurve(1.2, 0, 1.0, 0.15);
  public static ExpCurve steeringCurve = new ExpCurve(1.2, 0, -1.0, 0.15);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // create drivetrain from our file, utilizing the libary to do position
    // tracking, path following, and a couple of other tricks.
    dt = DrivetrainSubsystem.createSwerveModel();
    m_SwerveSubsystem = DrivetrainSubsystem.createSwerveSubsystem(dt);

    m_SwerveSubsystem.setDefaultCommand(new TeleopDriveCommand(m_SwerveSubsystem,
        () -> -getDriveY() * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS,
        () -> -getDriveX() * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS,
        () -> driver.rightStick.getX() * Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC));

        ShowInputs();

    Logger.getInstance().recordOutput("Pose Estimator", m_SwerveSubsystem.dt.getPose());
    Logger.getInstance().recordOutput("SwerveModuleStates", m_SwerveSubsystem.dt.getSwerveModuleStates());

    // Configure the button bindings
    configureButtonBindings();


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver.aButton.onTrue(new CalibrateGyro(m_SwerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public static double getDriveY() {
    return throttleCurve.calculateMappedVal(driver.leftStick.getY());
  }

  public static double getDriveX() {
    return throttleCurve.calculateMappedVal(driver.leftStick.getX());
  }

  public static double getDriveR() {
    double value = driver.rightStick.getY();
    value = steeringCurve.calculateMappedVal(value);
    return value;
  }
  public void ShowInputs(){
    master.addNumber("X Command", ()-> -getDriveX()*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    master.addNumber("Y Command", () -> -getDriveY() * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    master.addNumber("X Old Command", ()-> -getDriveX()*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS* Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    master.addNumber("Y Old Command", () -> -getDriveY() * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    
    master.addNumber("X Input", ()-> (-driver.leftStick.getX()*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS));
    master.addNumber("Y Input", () -> (-driver.leftStick.getY()*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS));
    
    master.addNumber("GyroReading", () -> dt.getGyroscopeRotation().getDegrees());
    
    
    master.addNumber("PoseX", ()-> m_SwerveSubsystem.dt.getPose().getX());
    master.addNumber("PoseY", ()-> m_SwerveSubsystem.dt.getPose().getY());
    master.addNumber("PoseRotation", ()-> m_SwerveSubsystem.dt.getPose().getRotation().getDegrees());

   

    
    
    
    
    
  }
}
