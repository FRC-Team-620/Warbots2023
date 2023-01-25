package frc.robot.util;

public class BabybotDrivePorts implements IDrivePorts {
    
    public final int leftFrontMotorCANId = 4;
    public final int rightFrontMotorCANId = 3;
    public final int leftRearMotorCANId = 2;
    public final int rightRearMotorCANId = 1;

    public final boolean rightFrontMotorInversion = false;
    public final boolean leftFrontMotorInversion = true;

    public final double wheelDiameterInInches = 4;

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
}
