// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final double kPDriveDistance = 0.01;
    public static final double kIDriveDistance = 0.01;
    public static final double kDDriveDistance = 0.0;

    public static final double kPKeepHeading = 0.01;
    public static final double kIKeepHeading = 0.0001;
    public static final double kDKeepHeading = 0.0;

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

  public static class PigeonConfigValues {
    public static final int pigeonCanID = 12;
    public static final double MountPoseYaw = 0;
    public static final double MountPosePitch = 0;
    public static final double MountPoseRoll = 0;

  }

}
