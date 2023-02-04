package frc.robot.util;

import edu.wpi.first.wpilibj.SPI;

public class BabybotDrivePorts implements IDrivePorts {
	// TODO:All PID loops require tuning for
	public final int leftFrontMotorCANId = 4;
	public final int rightFrontMotorCANId = 3;
	public final int leftRearMotorCANId = 2;
	public final int rightRearMotorCANId = 1;

	public final boolean rightFrontMotorInversion = false;
	public final boolean leftFrontMotorInversion = true;

	public final double wheelDiameterInInches = 4;

	public PIDConfig getDriveDistancePID = new PIDConfig(0.8, 0.01, 0.0);
	public PIDConfig getKeepHeadingPID = new PIDConfig(0.014, 0.01, 0.001);
	public PIDConfig getAutoDistancePID = new PIDConfig(50, 0.5, 0.0);

	public final double maxVelocity = 10;
	public final double maxAcceleration = 10;

	public final double balanceCreepSpeed = 0.1;

	public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);

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
