// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }
  public static class CANIdsTestBot {
    public static final int leftFrontMotorCANId = 4;
    public static final int rightFrontMotorCANId = 2;
    public static final int leftRearMotorCANId = 3;
    public static final int rightRearMotorCANId = 1;
  }
  public static class CANIdsMainBot {
    public static final int leftFrontMotorCANId = 4;
    public static final int rightFrontMotorCANId = 3;
    public static final int leftRearMotorCANId = 2;
    public static final int rightRearMotorCANId = 1;
  }
  public static class WheelConstants {
    public static final double conversionFactor = WheelConstants.gearRatio
            * Units.inchesToMeters(WheelConstants.wheelDiameterInInches)
            * Math.PI;
    public static final double wheelDiameterInInches = 4; 
    public static final double gearRatio = 0.12;
    

  }
   //All measurements in meters
  public static class ArenaConstants {
    public static final double kmidCubeHeight = Units.inchesToMeters(18.25);
    public static final double khighCubeHeight = Units.inchesToMeters(35.5);
    public static final double kmidCubeFromFront = Units.inchesToMeters(14.25);
    public static final double khighCubeFromFront = Units.inchesToMeters(31.625);

    public static final double kmidConeHeightMeters = Units.inchesToMeters(34);
    public static final double khighConeHeightMeters = Units.inchesToMeters(46);
    public static final double kmidConeFromFrontMeters = Units.inchesToMeters(22.75);  //how far the arm will have to reach to get to mid peg
    public static final double khighConeFromFront = Units.inchesToMeters(37.75);

    public static final double kmidTapeFromTopMeters = Units.inchesToMeters(8);
    public static final double khighTapeFromTopMeters = Units.inchesToMeters(3/16);
    public static final double ktapeLengthMeters = Units.inchesToMeters(4);


    public static final double substationShelfHeight = 37.75;
  }
  public static double kSimUpdateTime = 0.02;
  public static double kSimDrivekVLinear = 1.98;
  public static final double ksimDrivekALinear = 0.2;
  public static final double ksimDrivekVAngular = 1.5;
  public static final double kSimDrivekAAngular = 0.3;
  public static final double kSimTrackwidthMeters = 0.64;// 0.5207
}
