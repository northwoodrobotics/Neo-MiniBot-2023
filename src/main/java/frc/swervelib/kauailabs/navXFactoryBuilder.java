package frc.swervelib.kauailabs;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.Gyroscope;

public class navXFactoryBuilder {
    public Gyroscope build(AHRS navX) {
        return new GyroscopeImplementation(navX);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final AHRS navX;
        private final SimDouble angleSim;
        private static short[] AccelerationArray; 

        private static double gyroOffset = 0.0;

        private GyroscopeImplementation(AHRS navX) {
            this.navX = navX;

            int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            angleSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        }

        @Override
        public Rotation2d getGyroHeading() {
            if (navX.isMagnetometerCalibrated()) {
               // We will only get valid fused headings if the magnetometer is calibrated
               return Rotation2d.fromDegrees(navX.getFusedHeading() + gyroOffset);
            }
            // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
            return Rotation2d.fromDegrees(360.0 - navX.getYaw() + gyroOffset);
            
        }

        @Override
        public Boolean getGyroReady() {
            return !navX.isCalibrating();
        }
        // returns current Acceleration in m/s
        @Override 
        public Double getForwardAcceleration(){
            return (double) navX.getRawAccelX()*9.8;
        }
        @Override
        public short[] getAccelerlationArray(){
           AccelerationArray[0] = (short)navX.getWorldLinearAccelX();
           AccelerationArray[2] = (short)navX.getWorldLinearAccelY();
           AccelerationArray[3] = (short)navX.getWorldLinearAccelZ();
           return AccelerationArray;
        }

        @Override
        public void zeroGyroscope(double angle) {
            gyroOffset = angle - getGyroHeading().getDegrees();
        }
        @Override 
        public Double getGyroRoll(){
            return Double.valueOf(navX.getRoll());
        }

        @Override
        public void setAngle(double angle) {
            angleSim.set(angle);
        }
    }
}
