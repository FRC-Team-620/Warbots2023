package frc.robot.util;

import frc.robot.Constants;

public interface IDrivePorts {
  public static final int leftFrontMotorCANId = 4;
  public static final int rightFrontMotorCANId = 2;
  public static final int leftRearMotorCANId = 3;
  public static final int rightRearMotorCANId = 1;

  public static final boolean rightFrontMotorInversion = true;
  public static final boolean leftFrontMotorInversion = true;

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