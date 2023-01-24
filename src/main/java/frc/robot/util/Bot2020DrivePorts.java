package frc.robot.util;

public class Bot2020DrivePorts implements IDrivePorts {

    public final int leftFrontMotorCANId = 1;
    public final int rightFrontMotorCANId = 2;
    public final int leftRearMotorCANId = 3;
    public final int rightRearMotorCANId = 4;

    public final boolean rightFrontMotorInversion = true;
    public final boolean leftFrontMotorInversion = true;
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
}
