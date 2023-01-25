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

  public static IDrivePorts getDrivePorts(Constants.RobotType type) {
    switch (type) {
      case SUSAN:
        return new SusanDrivePorts();
      case BABY_BOT:
        return new BabybotDrivePorts();
      case BOT_2020:
        return new Bot2020DrivePorts();
      default:
        System.err.println("WARNING!: babybot defaulted");
        return new BabybotDrivePorts(); // TODO: Default to babybot
    }
  }
}