// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023;

import edu.wpi.first.math.util.Units;
import org.jmhsrobotics.frc2023.util.DetectRobot;
import org.jmhsrobotics.frc2023.util.IDrivePorts;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class RobotConstants {
		// IF YOU CHANGE EITHER OF THESE, YOU MUST CHANGE THE OTHER
		public static final double secondsPerTick = 0.02;
		// I just manually put 50.0 to avoid any floating-point errors
		public static final double ticksPerSecond = 50.0; // = 1.0 / 0.02
	}

	public static class DriveConstants {
		public static final double kPDriveDistance = 0.8;
		public static final double kIDriveDistance = 0.01;
		public static final double kDDriveDistance = 0.0;

		public static final double kPKeepHeading = 0.014;
		public static final double kIKeepHeading = 0.01;
		public static final double kDKeepHeading = 0.001;

		// public static final double metersPerEncoderTick = 0.03763;

		public static final double headingLockPIDOutputLimit = 0.3;

		public static final double angleSetBufferSeconds = 0.25;
		public static final double angleSetBufferTicks = DriveConstants.angleSetBufferSeconds
				* RobotConstants.ticksPerSecond;
	}

	public static class TurnAngleCommandConstants {

		// Basically a triangle profile
		public static final double maxAngularVelocity = 1000.0; // robot will not hit this
		public static final double maxAngularAcceleration = 200.0; // limiting factor
	}

	public static class OperatorConstants {
		public static final int driverControllerPort = 0;
		public static final int operatorControllerPort = 1;
	}

	public static class LEDConstants {
		public static final int LEDPWMPort = 9;
	}

	public static class RobotMathConstants {
		public static final double comparisonThreshhold = 0.005;
	}

	public enum RobotType {
		BABY_BOT, SUSAN, BOT_2020, UNKNOWN
	}

	public enum ControlMode {
		STOPPED, OPEN_LOOP, CLOSED_LOOP
	}

	public static enum ScoringType {
		CONE, CUBE
	}

	public enum GripperType {
		SOLENOID, MOTOR
	}

	public enum Setpoints {
		STOWED, FLOOR, MID, HIGH, PICKUP
	}

	public enum Direction {
		FORWARD, REVERSE, IN, OUT
	}

	public enum VisionPipeline {
		REFECTIVE_TAPE(0), APRIL_TAGS(1);

		VisionPipeline(int id) {
			this.id = id;
		}

		public final int id;
	}

	public static RobotType kRobotType = DetectRobot.identifyRobot(); // TODO: Better naming Scheme?

	public static GripperType kGripperType = GripperType.MOTOR;

	public static final IDrivePorts driveports = IDrivePorts.getDrivePorts(kRobotType);

	public static class WheelConstants {
		public static final double wheelDiameterInInches = driveports.getWheelDiameterInInches();
		public static final double conversionFactor = WheelConstants.gearRatio
				* Units.inchesToMeters(wheelDiameterInInches) * Math.PI;

		public static final double gearRatio = 0.12;

	}

	// All measurements in meters
	public static class ArenaConstants {
		public static final double kmidCubeHeightMeters = Units.inchesToMeters(18.25);
		public static final double khighCubeHeightMeters = Units.inchesToMeters(35.5);
		public static final double kmidCubeFromFrontMeters = Units.inchesToMeters(14.25);
		public static final double khighCubeFromFrontMeters = Units.inchesToMeters(31.625);

		public static final double kmidConeHeightMeters = Units.inchesToMeters(34);
		public static final double khighConeHeightMeters = Units.inchesToMeters(46);
		// how far the arm will have to reach to get to mid peg
		public static final double kmidConeFromFrontMeters = Units.inchesToMeters(22.75);
		public static final double khighConeFromFrontMeters = Units.inchesToMeters(37.75);

		public static final double kmidTapeFromTopMeters = Units.inchesToMeters(8);
		public static final double khighTapeFromTopMeters = Units.inchesToMeters(3 / 16);
		public static final double ktapeLengthMeters = Units.inchesToMeters(4);

		public static final double ksubstationShelfHeightMeters = Units.inchesToMeters(37.75);
		public static final double kchargeStationLengthMeters = Units.inchesToMeters(48);
	}

	public class AutoConstants {
		public static final double kPPitchSpeed = 0.1;
		public static final double kIPitchSpeed = 0.01;
		public static final double kDPitchSpeed = 0.001;

		public static final double kPRollCurvature = 0.014;
		public static final double kIRollCurvature = 0.01;
		public static final double kDRollCurvature = 0.001;

		public static final double maxVelocity = 2;
		public static final double maxAcceleration = 2;

		public static final double balanceCreepSpeed = 0.1;
		public static final double fineAdjustSpeed = 0.1;
		public static final double climbChargeStationSpeed = 0.24;
		public static final double balanceCenterLimitFromInitialTip = 1;
		public static final double onChargeStationAngle = 8;
		public static final double balancedAngle = 1;
		public static final double autoBalanceTimeoutSeconds = 30.0; // in seconds
	}

	public static class ArmConstants {
		public static final double armHeightMeters = Units.inchesToMeters(27.5);
		// public static final double maxEncoderCounts = 62.5;
		public static final double minExtensionLengthMillims = 0;// 77
		public static final double maxExtensionLengthMillims = 45;// 180
		// public static final double extensionMetersPerEncoderTick =
		// maxExtensionLengthMeters / maxEncoderCounts;
		public static final double armPitchGearRatio = 270;
		public static final double armMasskg = 7;
		public static final double pitchDegreesPerEncoderTick = -1.166;
		public static final double stowedDegrees = 20;
		public static final double minArmAngleDegrees = ArmConstants.stowedDegrees;
		public static final double maxArmAngleDegrees = 200;
		public static final double armDistanceToCenterMeters = Units.inchesToMeters(-8);

	}

	public static class WristConstants {

		public static final double degreesPerEncoderTick = 1.7108; // -0.02275025;
		public static final double encoderTicksAtZeroDegrees = 2342; // yes, exactly this 4690.0
		public static final double absotluteDegreePerEncoderTick = 277.008;
		public static final double absoluteAtZero = 0.3268;
		public static final double minPitchDegrees = -90;
		public static final double maxPitchDegrees = 145;
		
		public static final double relativeTicksAtMin = (minPitchDegrees - maxPitchDegrees) / degreesPerEncoderTick;
		public static final double stowedPositionRelative = WristConstants.maxPitchDegrees;
		public static final double stowedPositionAbsolute = WristConstants.stowedPositionRelative
				+ ArmConstants.stowedDegrees;
	}

	public static double kSimUpdateTime = 0.02;
	public static double kSimDrivekVLinear = 1.98;
	public static final double ksimDrivekALinear = 0.2;
	public static final double ksimDrivekVAngular = 1.5;
	public static final double kSimDrivekAAngular = 0.3;
	public static final double kSimTrackwidthMeters = 0.64;// 0.5207

	public static final boolean kCoastOnDisable = false;

	public class fudgeAngleConstant {
		public static final double limelightHeight = 1.02;// the height of the limelight sensor on the robot in meters
		public static final double targetHeight = 1.06;// the height of the high target in meters
		public static final double distance = 0.14;// distance between the limelight sensor and the center of the robot

	}
}
