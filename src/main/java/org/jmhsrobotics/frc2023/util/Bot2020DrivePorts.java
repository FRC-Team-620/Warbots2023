package org.jmhsrobotics.frc2023.util;

import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Bot2020DrivePorts implements IDrivePorts {

	public final int leftFrontMotorCANId = 1;
	public final int rightFrontMotorCANId = 2;
	public final int leftRearMotorCANId = 3;
	public final int rightRearMotorCANId = 4;

	public final boolean rightFrontMotorInversion = false;
	public final boolean leftFrontMotorInversion = true;

	public final double wheelDiameterInInches = 6;

	public final double maxVelocity = 2;
	public final double maxAcceleration = 2;

	public ProfiledPIDConfig autoDistanceProfiledPID = new ProfiledPIDConfig(2, 0.2, 0.0,
			new Constraints(maxVelocity, maxAcceleration));
	// spotless:off
	public PIDConfig keepHeadingPID = new PIDConfig(0.014, 0.01, 0.001);
	public PIDConfig balancingPID = new PIDConfig(0.4, 0.3, 0);
	// spotless:on

	// public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);
	public final IIMUWrapper imu = new PigeonIMU(30, getimConfiguration());
	public final double balanceCreepSpeed = 0.1;

	private Pigeon2Configuration getimConfiguration() {
		Pigeon2Configuration config = new Pigeon2Configuration();
		config.EnableCompass = false;
		return config;
	}

	@Override
	public ProfiledPIDConfig getAutoDistanceProfiledPID() {
		// TODO Auto-generated method stub
		return autoDistanceProfiledPID;
	}
	@Override
	public PIDConfig getKeepHeadingPID() {
		// TODO Auto-generated method stub
		return keepHeadingPID;
	}
	@Override
	public PIDConfig getBalancingPID() {
		// TODO Auto-generated method stub
		return balancingPID;
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
