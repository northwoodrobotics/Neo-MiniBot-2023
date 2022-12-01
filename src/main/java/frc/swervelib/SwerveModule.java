package frc.swervelib;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    ModuleConfiguration getModuleConfiguration();

    DriveController getDriveController();

    SteerController getSteerController();

    AbsoluteEncoder getAbsoluteEncoder();

    SwerveModulePosition getPosition();

    void resetWheelEncoder();

    void set(double driveVoltage, double steerAngle);

    void setVelocity(double driveVelocity, double steerAngle);
}
