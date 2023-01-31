package frc.robot.util;

import edu.wpi.first.wpilibj.SPI;

public class SusanDrivePorts implements IDrivePorts {

    public final int leftFrontMotorCANId = 4;
    public final int rightFrontMotorCANId = 2;
    public final int leftRearMotorCANId = 3;
    public final int rightRearMotorCANId = 1;

    public final boolean rightFrontMotorInversion = false;
    public final boolean leftFrontMotorInversion = true;

    public final double wheelDiameterInInches = 6;
    public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);

    public PIDConfig autoDistanceSusanPID = new PIDConfig(50, 0.5, 0.0);
    public PIDConfig DriveDistanceSusanPID = new PIDConfig(0.8, 0.01, 0.0);
    public PIDConfig KeepHeadingSusanPID = new PIDConfig(0.014, 0.01, 0.001);

    public final double maxVelocity = 10;
    public final double maxAcceleration = 10;

    public final double balanceCreepSpeed = 0.1;

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
        return autoDistanceSusanPID.kp;
    }
    @Override
    public double getAutoDistanceKI() {
        // TODO Auto-generated method stub
        return autoDistanceSusanPID.ki;
    }
    @Override
    public double getAutoDistanceKD() {
        // TODO Auto-generated method stub
        return autoDistanceSusanPID.kd;
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
        return DriveDistanceSusanPID.kp;
    }
    @Override
    public double getkIDriveDistance() {
        // TODO Auto-generated method stub
        return DriveDistanceSusanPID.ki;
    }
    @Override
    public double getkDDriveDistance() {
        // TODO Auto-generated method stub
        return DriveDistanceSusanPID.kd;
    }
    @Override
    public double getkPKeepHeading() {
        // TODO Auto-generated method stub
        return KeepHeadingSusanPID.kp;
    }
    @Override
    public double getkIKeepHeading() {
        // TODO Auto-generated method stub
        return KeepHeadingSusanPID.ki;
    }
    @Override
    public double getkDKeepHeading() {
        // TODO Auto-generated method stub
        return KeepHeadingSusanPID.kd;
    }
    @Override
    public IIMUWrapper getIMU() {
        return imu;
    }
}
