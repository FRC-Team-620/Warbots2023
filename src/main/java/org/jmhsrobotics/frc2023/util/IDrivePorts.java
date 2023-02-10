package org.jmhsrobotics.frc2023.util;

import org.jmhsrobotics.frc2023.Constants;

public interface IDrivePorts {
	public int getLeftFrontMotorCANId();
	public int getRightFrontMotorCANId();
	public int getLeftRearMotorCANId();
	public int getRightRearMotorCANId();

	public boolean getRightFrontMotorInversion();
	public boolean getLeftFrontMotorInversion();

	public double getWheelDiameterInInches();

	public ProfiledPIDConfig getAutoDistanceProfiledPID();
	public PIDConfig getKeepHeadingPID();
	public PIDConfig getBalancingPID();

	public double getMaxVelocity();
	public double getMaxAcceleration();

	public double getBalanceCreepSpeed();

	public IIMUWrapper getIMU();

	public static IDrivePorts getDrivePorts(Constants.RobotType type) {
		switch (type) {
			case SUSAN :
				return new SusanDrivePorts();
			case BABY_BOT :
				return new BabybotDrivePorts();
			case BOT_2020 :
				return new Bot2020DrivePorts();
			default :
				System.err.println("WARNING!: DEFAULTED TO BABYBOT");
				return new BabybotDrivePorts(); // TODO: Default to babybot
		}
	}
}
