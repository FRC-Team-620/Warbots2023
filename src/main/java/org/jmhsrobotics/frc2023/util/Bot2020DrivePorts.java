package org.jmhsrobotics.frc2023.util;

import com.ctre.phoenix.sensors.Pigeon2Configuration;

public class Bot2020DrivePorts implements IDrivePorts {

	public final int leftFrontMotorCANId = 1;
	public final int rightFrontMotorCANId = 2;
	public final int leftRearMotorCANId = 3;
	public final int rightRearMotorCANId = 4;

	public final boolean rightFrontMotorInversion = false;
	public final boolean leftFrontMotorInversion = true;

	public final double wheelDiameterInInches = 6;

	public PIDConfig getAutoDistancePID = new PIDConfig(50, 0.5, 0.0);
	public PIDConfig getDriveDistancePID = new PIDConfig(0.8, 0.01, 0.0);
	public PIDConfig getKeepHeadingPID = new PIDConfig(0.010, 0.010, 0);

	public final double maxVelocity = 10;
	public final double maxAcceleration = 10;

	public static final double kPDriveDistance = 0.8;
	public static final double kIDriveDistance = 0.01;
	public static final double kDDriveDistance = 0.0;

	public static final double kPKeepHeading = 0.010;
	public static final double kIKeepHeading = 0.010;
	public static final double kDKeepHeading = 0;
	// public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);
	public final IIMUWrapper imu = new PigeonIMU(30, getimConfiguration());
	public final double balanceCreepSpeed = 0.1;

	private Pigeon2Configuration getimConfiguration() {
		Pigeon2Configuration config = new Pigeon2Configuration();
		config.EnableCompass = false;
		return config;
	}

	@Override
	public PIDConfig getAutoDistancePID() {
		// TODO Auto-generated method stub
		return getAutoDistancePID;
	}
	@Override
	public PIDConfig getDriveDistancePID() {
		// TODO Auto-generated method stub
		return getDriveDistancePID;
	}
	@Override
	public PIDConfig getKeepHeadingPID() {
		// TODO Auto-generated method stub
		return getKeepHeadingPID;
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
	public double getMaxVelocity() {
		// TODO Auto-generated method stub
		return maxVelocity;
	}
	@Override
	public double getMaxAcceleration() {
		// TODO Auto-generated method stub
		return maxAcceleration;
	}
	@Override
	public double getBalanceCreepSpeed() {
		// TODO Auto-generated method stub
		return balanceCreepSpeed;
	}

	@Override
	public IIMUWrapper getIMU() {
		return imu;
	}
}
