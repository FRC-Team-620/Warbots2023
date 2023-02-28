package org.jmhsrobotics.frc2023.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.Robot;
import org.jmhsrobotics.frc2023.RobotMath;
import org.jmhsrobotics.frc2023.RobotMath.DiminishingAverageHandler;
import org.jmhsrobotics.frc2023.util.IIMUWrapper;
import org.jmhsrobotics.frc2023.util.vision.PhotonManager;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelemetrySubsystem extends SubsystemBase {

	private AprilTagFieldLayout layout;
	private PhotonPoseEstimator currentEstimator;
	private Drivetrain drivetrain;

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

	private IIMUWrapper imu;
	private IMUState imuState;
	private Rotation2d imuRotation2d;

	private DiminishingAverageHandler yawAngularVelocityHandler;
	private DiminishingAverageHandler pitchAngularVelocityHandler;
	private DiminishingAverageHandler rollAngularVelocityHandler;

	public TelemetrySubsystem(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		this.imu = Constants.driveports.getIMU();
		this.imuState = new IMUState();
		this.imuRotation2d = this.imu.getRotation2d();
		this.yawAngularVelocityHandler = new DiminishingAverageHandler(0.5);
		this.pitchAngularVelocityHandler = new DiminishingAverageHandler(0.5);
		this.rollAngularVelocityHandler = new DiminishingAverageHandler(0.5);
		// PhotonManager.getInstance().mainCam

		AprilTagFieldLayout tmp = null;
		try {
			tmp = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getAbsolutePath() + "/2023-chargedup.json");
			currentEstimator = new PhotonPoseEstimator(tmp, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
					PhotonManager.getInstance().mainCam, new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()));
		} catch (IOException e) {
			DriverStation.reportError("Failed to load Vision targets!", false);
			e.printStackTrace();
		}
		layout = tmp;

	}

	@Override
	public void periodic() {

		if (currentEstimator != null) {
			PhotonCamera mainCam = PhotonManager.getInstance().mainCam;
			if (mainCam.getPipelineIndex() == Constants.VisionPipeline.APRIL_TAGS.id) {
				currentEstimator.setReferencePose(drivetrain.getPose());
				Optional<EstimatedRobotPose> estPoses = currentEstimator.update();
				if (estPoses.isPresent()) {
					// estPoses.get().estimatedPose
					Robot.field.getObject("VisionEst").setPose(estPoses.get().estimatedPose.toPose2d());
					EstimatedRobotPose estpose = estPoses.get();

				}
			}
		}

		this.imuRotation2d = this.imu.getRotation2d();

		IMUState previousState = new IMUState(this.imuState);

		this.imuState.yaw = this.imu.getYaw();
		this.imuState.pitch = this.imu.getPitch();
		this.imuState.roll = this.imu.getRoll();

		double relativeChange;

		// spotless:off
		relativeChange = RobotMath.relativeAngle(previousState.yaw, this.imuState.yaw);
		this.imuState.yawVelocity = this.yawAngularVelocityHandler.feed(
				relativeChange / Constants.RobotConstants.secondsPerTick);

		relativeChange = RobotMath.relativeAngle(previousState.pitch, this.imuState.pitch);
		this.imuState.pitchVelocity = this.pitchAngularVelocityHandler.feed(
				relativeChange / Constants.RobotConstants.secondsPerTick);

		relativeChange = RobotMath.relativeAngle(previousState.roll, this.imuState.roll);
		this.imuState.rollVelocity = this.rollAngularVelocityHandler.feed(
				relativeChange / Constants.RobotConstants.secondsPerTick);
		// spotless:on

		SmartDashboard.putNumber("Telemetry/yaw", this.imuState.yaw);
		SmartDashboard.putNumber("Telemetry/pitch", this.imuState.pitch);
		SmartDashboard.putNumber("Telemetry/roll", this.imuState.roll);
		SmartDashboard.putNumber("Telemetry/yawVelocity", this.imuState.yawVelocity);
		SmartDashboard.putNumber("Telemetry/pitchVelocity", this.imuState.pitchVelocity);
		SmartDashboard.putNumber("Telemetry/rollVelocity", this.imuState.rollVelocity);

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
