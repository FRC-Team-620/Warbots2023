package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.RobotMath.DiminishingAverageHandler;
import org.jmhsrobotics.frc2023.util.IIMUWrapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelemetrySubsystem extends SubsystemBase {

	public static class IMUState {
		public double yaw;
		public double pitch;
		public double roll;
		/** angular (rotational) velocity */
		public double yawVelocity;
		/** angular (rotational) velocity */
		public double pitchVelocity;
		/** angular (rotational) velocity */
		public double rollVelocity;
	}

	private IIMUWrapper imu = Constants.driveports.getIMU();
	private IMUState imuState;
	private Rotation2d imuRotation2d;

	private DiminishingAverageHandler yawAngularVelocityHandler;
	private DiminishingAverageHandler pitchAngularVelocityHandler;
	private DiminishingAverageHandler rollAngularVelocityHandler;

	public TelemetrySubsystem() {

		this.yawAngularVelocityHandler = new DiminishingAverageHandler(0.5);
		this.pitchAngularVelocityHandler = new DiminishingAverageHandler(0.5);
		this.rollAngularVelocityHandler = new DiminishingAverageHandler(0.5);
	}

	@Override
	public void periodic() {

		this.imuRotation2d = this.imu.getRotation2d();

		IMUState previousState = this.imuState;

		this.imuState.yaw = this.imu.getYaw();
		this.imuState.pitch = this.imu.getPitch();
		this.imuState.roll = this.imu.getRoll();

		// spotless:off
		this.imuState.yawVelocity = this.yawAngularVelocityHandler.feed(
            (this.imuState.yaw - previousState.yaw) / Constants.RobotConstants.secondsPerTick
        );

		this.imuState.pitchVelocity = this.pitchAngularVelocityHandler.feed(
            (this.imuState.pitch - previousState.pitch) / Constants.RobotConstants.secondsPerTick
        );

		this.imuState.rollVelocity = this.rollAngularVelocityHandler.feed(
            (this.imuState.roll - previousState.roll) / Constants.RobotConstants.secondsPerTick
        );
        // spotless:on
	}

	public IMUState getIMUState() {
		return this.imuState;
	}

	public Rotation2d getRotation2d() {
		return this.imuRotation2d;
	}

	public double getYaw() {
		return this.imuState.yaw;
	}

	public double getPitch() {
		return this.imuState.pitch;
	}

	public double getRoll() {
		return this.imuState.roll;
	}

	public double getYawVelocity() {
		return this.imuState.yawVelocity;
	}

	public double getPitchVelocity() {
		return this.imuState.pitchVelocity;
	}

	public double getRollVelocity() {
		return this.imuState.rollVelocity;
	}

}
