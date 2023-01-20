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
    public static final int leftFrontMotorCANId = 1;
    public static final int rightFrontMotorCANId = 2;
    public static final int leftRearMotorCANId = 3;
    public static final int rightRearMotorCANId = 4;
  }
  public static class WheelConstants {
    public static final double conversionFactor = WheelConstants.gearRatio
            * Units.inchesToMeters(WheelConstants.wheelDiameterInInches)
            * Math.PI;
    public static final double wheelDiameterInInches = 4; 
    public static final double gearRatio = 0.12;
    

  }
public static double kSimUpdateTime = 0.02;
public static double kSimDrivekVLinear = 1.98;
public static final double ksimDrivekALinear = 0.2;
public static final double ksimDrivekVAngular = 1.5;
public static final double kSimDrivekAAngular = 0.3;
public static final double kSimTrackwidthMeters = 0.64;// 0.5207
}
