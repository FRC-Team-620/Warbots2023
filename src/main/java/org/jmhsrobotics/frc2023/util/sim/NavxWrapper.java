package org.jmhsrobotics.frc2023.util.sim;

import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class NavxWrapper {
	private final IGyroWrapper mYawGyro;
	private final IGyroWrapper mPitchGyro;
	private final IGyroWrapper mRollGyro;

	public NavxWrapper() {
		SimDeviceSim deviceSim = new SimDeviceSim("navX-Sensor[0]");

		mYawGyro = new BaseGyroWrapper(deviceSim.getDouble("Yaw"));
		mPitchGyro = new BaseGyroWrapper(deviceSim.getDouble("Pitch"));
		mRollGyro = new BaseGyroWrapper(deviceSim.getDouble("Roll"));
	}

	public IGyroWrapper getYawGyro() {
		return mYawGyro;
	}

	public IGyroWrapper getPitchGyro() {
		return mPitchGyro;
	}

	public IGyroWrapper getRollGyro() {
		return mRollGyro;
	}
}
