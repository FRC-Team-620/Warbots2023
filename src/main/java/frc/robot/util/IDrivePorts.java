package frc.robot.util;

import frc.robot.Constants;

public interface IDrivePorts {
    public static final int leftFrontMotorCANId = 4;
    public static final int rightFrontMotorCANId = 2;
    public static final int leftRearMotorCANId = 3;
    public static final int rightRearMotorCANId = 1;



    public static IDrivePorts getDrivePorts(Constants.RobotType type){
      switch(type){
        case Constants.RobotType.SUSAN:
          return new SusanDrivePorts();
        break;
        case Constants.RobotType.BABY_BOT:
          return new BabybotDrivePorts();
        case Constants.RobotType.2020_BOT: 
          return new Bot2020DrivePorts();
        default:
          System.out.println("asdf");
        break;
      }
    }
  }