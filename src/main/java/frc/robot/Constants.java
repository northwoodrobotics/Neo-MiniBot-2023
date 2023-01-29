// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.ExternalLib.PoofLib.util.InterpolatingDouble;
import frc.ExternalLib.PoofLib.util.InterpolatingTreeMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean TuningMode = false;
    public static final double MinVoltage = 8.0;

    public static final class VisionConstants {
        public static final double TargetHeight = Units.inchesToMeters(102);
        public static final double blindlightHeight = Units.inchesToMeters(40);
        public static final double blindlightAngle = 23;
        public static final Pose2d GoalPose = new Pose2d(4, 6, Rotation2d.fromDegrees(0));
        public static final double intakeCamHeight = Units.inchesToMeters(38);

    }

    public static final class DriveConstants {
        public static final class AimConstants {
            public static final double AimP = 0.006;
            public static final double AimI = 0.005;
            public static final double AimD = 0.0027;
        }

        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(15.5); // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double WHEELBASE_METERS = Units.inchesToMeters(15.5); // FIXME Measure and set wheelbase
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));
        public static final double WHEEL_DIAMETER_METERS = 0.10033; // .10033 = ~4 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final int PIGEON_ID = 28; // FIXME Set Pigeon ID
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 6; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(5.80); // FIXME Measure and
                                                                                                    // set front left
                                                                                                    // steer offset
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(178.33+227.72); // FIXME Measure and set
                                                                                                // front right steer
                                                                                                // offse
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 10; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(182.28+167.16); // FIXME Measure and set
                                                                                             // back left steer offset
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 15; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(186.06+ 224.38); // FIXME Measure and set
                                                                                             // back right steer offset
        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(4);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(4);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(90);
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS / 0.25; // 0-full time of 0.25
                                                                                            // second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC / 0.25; // 0-full time
                                                                                                         // of 0.25
                                                                                                         // second
        public static final double MAX_VOLTAGE = 12.0; // Maximum Voltage sent to the drive motors
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SENSOR CONSTANTS
        // Sensor-related constants - pulled from datasheets for the sensors and
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////// gearboxes

        static public final Pose2d DFLT_START_POSE = new Pose2d(Units.feetToMeters(24.0), Units.feetToMeters(10.0),
                Rotation2d.fromDegrees(0));
        static public final double ROBOT_MASS_kg = Units.lbsToKilograms(30);
        static public final double ROBOT_MOI_KGM2 = 1.0 / 12.0 * ROBOT_MASS_kg * Math.pow((WHEELBASE_METERS * 1.1), 2)
                * 2;
        public static final double MASS_kg = Units.lbsToKilograms(30);
        public static final double MOI_KGM2 = 1.0 / 12.0 * MASS_kg * Math.pow((TRACKWIDTH_METERS * 1.1), 2) * 2;
        // degrees per second
        public static final double Min_Rotation_Deg = 25;
    }

    public final static class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double TRAJECTORYXkP = 1;
        public static final double TRAJECTORYXkI = 0;
        public static final double TRAJECTORYXkD = 0;
        public static final double TRAJECTORYYkP = 1;
        public static final double TRAJECTORYYkI = 0;
        public static final double TRAJECTORYYkD = 0;
        public static final double DriveKS = 1.1152;
        public static final double DriveKV = 0.62013;
        public static final double DriveKA = 0.12412;
        public static final double THETACONTROLLERkP = 1;
        public static final double THETACONTROLLERkI = 0;
        public static final double THETACONTROLLERkD = 0;
        // FIXME In order for auto to work consistently and as viewed on the path
        // software
        // Tape Measure out 1 Meter and Drive back and forth on it, change this number
        // until odometry says 0-1m.
        public static final double TractionConstant = Units.feetToMeters(13.50);

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ClimberConstants{
        public static final int ClimbLeftID = 28; 
        public static final int ClimbRightID = 29; 
        public static final int ClimbSolenoidForward= 0;
        public static final int CLimbSolenoidReverse = 1;

        public static final double Climb1P = 0.5;
        public static final double Climb1D = 0.0;
        public static final double Climb1I = 0.0;
        public static final double Climb1F = 0.045;
        public static final double Climb1MotionAccel = 8192.0; 
        public static final double Climb1MotionVelocity =8192.0; 
        public static final double Climb2P = 0.5;
        public static final double Climb2F = 0.045;
        public static final double Climb2D = 0.0;
        public static final double Climb2I = 0.0;
        public static final double Climb2MotionAccel = 8192.0; 
        public static final double Climb2MotionVelocity =8192.0; 
        public static final float Climb1SoftForward = 0;
        public static final float Climb1SoftReverse = 0;
        public static final float Climb2SoftForward = 0;
        public static final float Climb2SoftReverse = 0;
        public static final double Climb1GearRatio = 20.0;
        public static final double Climb2GearRatio = 20.0; 
    }

}
