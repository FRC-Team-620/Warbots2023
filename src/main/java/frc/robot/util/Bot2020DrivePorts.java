package frc.robot.util;

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
    @Override
    public int getLeftFrontMotorCANId() {
        // TODO Auto-generated method stub
        return leftFrontMotorCANId;
    }
    @Override
    public int getRightFrontMotorCANId() {
        // TODO Auto-generated method stub
       return rightFrontMotorCANId;
    }
    @Override
    public int getLeftRearMotorCANId() {
        // TODO Auto-generated method stub
        return leftRearMotorCANId;
    }
    @Override
    public int getRightRearMotorCANId() {
        // TODO Auto-generated method stub
        return rightRearMotorCANId;
    }
    @Override
    public boolean getRightFrontMotorInversion() {
        // TODO Auto-generated method stub
        return rightFrontMotorInversion;
    }
    @Override
    public boolean getLeftFrontMotorInversion() {
        // TODO Auto-generated method stub
        return leftFrontMotorInversion;
    }
    @Override
    public double getWheelDiameterInInches() {
        // TODO Auto-generated method stub
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
}
