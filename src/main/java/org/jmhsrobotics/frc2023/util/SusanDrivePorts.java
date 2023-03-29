package org.jmhsrobotics.frc2023.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.SPI;

public class SusanDrivePorts implements IDrivePorts {

	public final int leftFrontMotorCANId = 4;
	public final int rightFrontMotorCANId = 2;
	public final int leftRearMotorCANId = 3;
	public final int rightRearMotorCANId = 1;

	public final boolean rightFrontMotorInversion = false;
	public final boolean leftFrontMotorInversion = true;

	public final double wheelDiameterInInches = 6;
	public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);

	public final double driveOpenLoopRampRate = 0.2;

	public final double maxVelocity = 2;
	public final double maxAcceleration = 2;

	public ProfiledPIDConfig autoDistanceProfiledPID = new ProfiledPIDConfig(2, 0.2, 0.0,
			new Constraints(maxVelocity, maxAcceleration));
	public PIDConfig keepHeadingPID = new PIDConfig(0.010, 0.010, 0);
	public PIDConfig balancingPID = new PIDConfig(0.4, 0.2, 0);
	public PIDConfig turnDeltaAnglePID = new PIDConfig(0.014, 0.06, 0.001);

	public final double balanceCreepSpeed = 0.1;

	@Override
	public ProfiledPIDConfig getAutoDistanceProfiledPID() {
		return autoDistanceProfiledPID;
	}
	@Override
	public PIDConfig getKeepHeadingPID() {
		return keepHeadingPID;
	}
	@Override
	public PIDConfig getBalancingPID() {
		return balancingPID;
	}
	@Override
	public PIDConfig getTurnDeltaAnglePID() {
		return turnDeltaAnglePID;
	}

	@Override
	public int getLeftFrontMotorCANId() {
		return leftFrontMotorCANId;
	}
	@Override
	public int getRightFrontMotorCANId() {
		return rightFrontMotorCANId;
	}
	@Override
	public int getLeftRearMotorCANId() {
		return leftRearMotorCANId;
	}
	@Override
	public int getRightRearMotorCANId() {
		return rightRearMotorCANId;
	}

	@Override
	public int getArmAngleCANId() {
		return 5;
	}

	@Override
	public int getArmExtensionCANId() {
		return 6;
	}

	@Override
	public int getIntakeCANId() {
		return 7;
	}

	@Override
	public int getWristCANId() {
		return 8;
	}

	@Override
	public int getWristAbsEncoderCANId() {
		return 9;
	}

	@Override
	public int getGrabberIntakeSolenoidId() {
		return 2;
	}

	@Override
	public boolean getRightFrontMotorInversion() {
		return rightFrontMotorInversion;
	}
	@Override
	public boolean getLeftFrontMotorInversion() {
		return leftFrontMotorInversion;
	}
	@Override
	public double getWheelDiameterInInches() {
		return wheelDiameterInInches;
	}

	@Override
	public double getDriveOpenLoopRampRate() {
		return driveOpenLoopRampRate;
	}

	@Override
	public double getMaxVelocity() {
		return maxVelocity;
	}
	@Override
	public double getMaxAcceleration() {
		return maxAcceleration;
	}
	@Override
	public double getBalanceCreepSpeed() {
		return balanceCreepSpeed;
	}

	@Override
	public IIMUWrapper getIMU() {
		return imu;
	}
}
