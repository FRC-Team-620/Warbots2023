// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.util.DetectRobot;
import frc.robot.util.IDrivePorts;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class RobotConstants {
    public static final double secondsPerTick = 0.02;
  }

  public static class DriveConstants {
    public static final double kPDriveDistance = 0.8;
    public static final double kIDriveDistance = 0.01;
    public static final double kDDriveDistance = 0.0;

    public static final double kPKeepHeading = 0.014;
    public static final double kIKeepHeading = 0.01;
    public static final double kDKeepHeading = 0.001;

    public static final double metersPerEncoderTick = 0.03763;

    public static final double angleSetBufferSeconds = 0.25;
    public static final double angleSetBufferTicks = 
      DriveConstants.angleSetBufferSeconds / RobotConstants.secondsPerTick;
  }
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
	public static final int operatorControllerPort = 1;
  }

  public static class RobotMathConstants {
    public static final double comparisonThreshhold = 0.005;
  }
  
  public enum RobotType {
    BABY_BOT, SUSAN, BOT_2020, UNKNOWN
}
  public static RobotType kRobotType = DetectRobot.identifyRobot(); //TODO: Better naming Scheme?

  public static final IDrivePorts driveports = IDrivePorts.getDrivePorts(kRobotType);

  public static class WheelConstants {
    public static final double wheelDiameterInInches = 4;    
    public static final double conversionFactor = WheelConstants.gearRatio * Units.inchesToMeters(wheelDiameterInInches)
            * Math.PI;

    public static final double gearRatio = 0.12;
    
 }
	//All measurements in meters
  public static class ArenaConstants {
    public static final double kmidCubeHeightMeters = Units.inchesToMeters(18.25);
    public static final double khighCubeHeightMeters = Units.inchesToMeters(35.5);
    public static final double kmidCubeFromFrontMeters = Units.inchesToMeters(14.25);
    public static final double khighCubeFromFrontMeters = Units.inchesToMeters(31.625);

    public static final double kmidConeHeightMeters = Units.inchesToMeters(34);
    public static final double khighConeHeightMeters = Units.inchesToMeters(46);
    public static final double kmidConeFromFrontMeters = Units.inchesToMeters(22.75);  //how far the arm will have to reach to get to mid peg
    public static final double khighConeFromFrontMeters = Units.inchesToMeters(37.75);

    public static final double kmidTapeFromTopMeters = Units.inchesToMeters(8);
    public static final double khighTapeFromTopMeters = Units.inchesToMeters(3/16);
    public static final double ktapeLengthMeters = Units.inchesToMeters(4);


    public static final double ksubstationShelfHeightMeters = Units.inchesToMeters(37.75);
    public static final double kchargeStationLengthMeters = Units.inchesToMeters(48);
}
