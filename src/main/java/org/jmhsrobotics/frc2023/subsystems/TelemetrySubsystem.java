package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.RobotMath;
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

		public IMUState() {
		}

		/**
		 * Copy constructor.
		 *
		 * @param other
		 *            The IMUState to be copied from.
		 */
		public IMUState(IMUState other) {

			this.yaw = other.yaw;
			this.pitch = other.pitch;
			this.roll = other.roll;
			this.yawVelocity = other.yawVelocity;
			this.pitchVelocity = other.pitchVelocity;
			this.rollVelocity = other.rollVelocity;
		}
	}

	private IIMUWrapper imu = Constants.driveports.getIMU();
	private IMUState imuState = new IMUState();
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

		IMUState previousState = new IMUState(this.imuState);

		this.imuState.yaw = this.imu.getYaw();
		this.imuState.pitch = this.imu.getPitch();
		this.imuState.roll = this.imu.getRoll();

		double relativeChange;

		// spotless:off
        relativeChange = RobotMath.relativeAngle(previousState.yaw, this.imuState.yaw);
		this.imuState.yawVelocity = this.yawAngularVelocityHandler.feed(
            relativeChange / Constants.RobotConstants.secondsPerTick
        );

        relativeChange = RobotMath.relativeAngle(previousState.pitch, this.imuState.pitch);
		this.imuState.pitchVelocity = this.pitchAngularVelocityHandler.feed(
            relativeChange / Constants.RobotConstants.secondsPerTick
        );

        relativeChange = RobotMath.relativeAngle(previousState.roll, this.imuState.roll);
		this.imuState.rollVelocity = this.rollAngularVelocityHandler.feed(
            relativeChange / Constants.RobotConstants.secondsPerTick
        );
        // spotless:on
	}

	public IMUState getIMUState() {
		return new IMUState(this.imuState);
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
