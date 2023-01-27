package frc.robot.util;

import edu.wpi.first.wpilibj.SPI;

public class Bot2020DrivePorts implements IDrivePorts {

    public final int leftFrontMotorCANId = 1;
    public final int rightFrontMotorCANId = 2;
    public final int leftRearMotorCANId = 3;
    public final int rightRearMotorCANId = 4;

    public final boolean rightFrontMotorInversion = true;
    public final boolean leftFrontMotorInversion = true;

    public final double wheelDiameterInInches = 6;

    public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);
    // public final IIMUWrapper imu = new PigeonIMU(10); // TODO: When you have a pigeon installed
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
    public IIMUWrapper getIMU() {
        return imu;
    }
}
