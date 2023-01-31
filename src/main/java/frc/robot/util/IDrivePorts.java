package frc.robot.util;

import frc.robot.Constants;

public interface IDrivePorts {
  public int getLeftFrontMotorCANId();
  public int getRightFrontMotorCANId();
  public int getLeftRearMotorCANId();
  public int getRightRearMotorCANId();

  public boolean getRightFrontMotorInversion();
  public boolean getLeftFrontMotorInversion();

  public double getWheelDiameterInInches();

  public double getAutoDistanceKP();
  public double getAutoDistanceKI();
  public double getAutoDistanceKD();

  public double getMaxVelocity();
  public double getMaxAcceleration();

  public double getBalanceCreepSpeed();

  public double getkPDriveDistance();
  public double getkIDriveDistance(); 
  public double getkDDriveDistance();

  public double getkPKeepHeading();
  public double getkIKeepHeading();
  public double getkDKeepHeading();

  public IIMUWrapper getIMU();

  public static IDrivePorts getDrivePorts(Constants.RobotType type) {
    switch (type) {
      case SUSAN:
        return new SusanDrivePorts();
      case BABY_BOT:
        return new BabybotDrivePorts();
      case BOT_2020:
        return new Bot2020DrivePorts();
      default:
        System.err.println("WARNING!: DEFAULTED TO BABYBOT");
        return new BabybotDrivePorts(); // TODO: Default to babybot
    }
  }
}