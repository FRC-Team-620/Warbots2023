package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.wpilibj.SPI;

public class Bot2020DrivePorts implements IDrivePorts {
    
    public final int leftFrontMotorCANId = 1;
    public final int rightFrontMotorCANId = 2;
    public final int leftRearMotorCANId = 3;
    public final int rightRearMotorCANId = 4;

    public final boolean rightFrontMotorInversion = true;
    public final boolean leftFrontMotorInversion = true;

    public final double wheelDiameterInInches = 6;

    public final double autoDistanceKP = 50;
    public final double autoDistanceKI = 0.5;
    public final double autoDistanceKD = 0.0;

    public final double maxVelocity = 10;
    public final double maxAcceleration = 10;

    public final double balanceCreepSpeed = 0.1;

    public static final double kPDriveDistance = 0.8;
    public static final double kIDriveDistance = 0.01;
    public static final double kDDriveDistance = 0.0;
    
    public static final double kPKeepHeading = 0.010;
    public static final double kIKeepHeading = 0.010;
    public static final double kDKeepHeading = 0;
    //public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);
    public final IIMUWrapper imu = new PigeonIMU(30, getimConfiguration()); 

    private Pigeon2Configuration getimConfiguration() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.EnableCompass = false;
        return config;
    }
    @Override
    public int getLeftFrontMotorCANId() {
        return leftFrontMotorCANId;
    }
    @Override
    public int getRightFrontMotorCANId() {
       return rightFrontMotorCANId;
    }
    @Override
    public int getLeftRearMotorCANId() {
        return leftRearMotorCANId;
    }
    @Override
    public int getRightRearMotorCANId() {
        return rightRearMotorCANId;
    }
    @Override
    public boolean getRightFrontMotorInversion() {
        return rightFrontMotorInversion;
    }
    @Override
    public boolean getLeftFrontMotorInversion() {
        return leftFrontMotorInversion;
    }
    @Override
    public double getWheelDiameterInInches() {
        return wheelDiameterInInches;
    }
    @Override
    public double getAutoDistanceKP() {
        // TODO Auto-generated method stub
        return autoDistanceKP;
    }
    @Override
    public double getAutoDistanceKI() {
        // TODO Auto-generated method stub
        return autoDistanceKI;
    }
    @Override
    public double getAutoDistanceKD() {
        // TODO Auto-generated method stub
        return autoDistanceKD;
    }
    @Override
    public double getMaxVelocity() {
        // TODO Auto-generated method stub
        return maxVelocity;
    }
    @Override
    public double getMaxAcceleration() {
        // TODO Auto-generated method stub
        return maxAcceleration;
    }
    @Override
    public double getBalanceCreepSpeed() {
        // TODO Auto-generated method stub
        return balanceCreepSpeed;
    }
    @Override
    public double getkPDriveDistance() {
        // TODO Auto-generated method stub
        return kPDriveDistance;
    }
    @Override
    public double getkIDriveDistance() {
        // TODO Auto-generated method stub
        return kIDriveDistance;
    }
    @Override
    public double getkDDriveDistance() {
        // TODO Auto-generated method stub
        return kDDriveDistance;
    }
    @Override
    public double getkPKeepHeading() {
        // TODO Auto-generated method stub
        return kPKeepHeading;
    }
    @Override
    public double getkIKeepHeading() {
        // TODO Auto-generated method stub
        return kIKeepHeading;
    }
    @Override
    public double getkDKeepHeading() {
        // TODO Auto-generated method stub
        return kDKeepHeading;
    }
    @Override
    public IIMUWrapper getIMU() {
        return imu;
    }
}
