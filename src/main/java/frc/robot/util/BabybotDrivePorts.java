package frc.robot.util;

import edu.wpi.first.wpilibj.SPI;

public class BabybotDrivePorts implements IDrivePorts {
    
    public final int leftFrontMotorCANId = 4;
    public final int rightFrontMotorCANId = 3;
    public final int leftRearMotorCANId = 2;
    public final int rightRearMotorCANId = 1;

    public final boolean rightFrontMotorInversion = false;
    public final boolean leftFrontMotorInversion = true;

    public final double wheelDiameterInInches = 4;
    public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);
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
