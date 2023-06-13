package org.jmhsrobotics.frc2023.commands.auto;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrajectoryFollowingCommand extends CommandBase {

	private Drivetrain drivetrain;
	private Trajectory trajectory;
	private Timer timer;

	private PIDController rightSpeedController = new PIDController(0, 0, 0);
	private PIDController leftSpeedController = new PIDController(0, 0, 0);
	RamseteController ramsetController = new RamseteController();
	private DifferentialDriveKinematics kine = new DifferentialDriveKinematics(Constants.kSimTrackwidthMeters);

	public TrajectoryFollowingCommand(Drivetrain drivetran, Trajectory trajectory) {
		this.drivetrain = drivetran;
		this.trajectory = trajectory;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		timer = new Timer();
		timer.start();
	}

	@Override
	public void execute() {
		var targetWheelSpeed = kine
				.toWheelSpeeds(ramsetController.calculate(drivetrain.getPose(), trajectory.sample(timer.get())));
		var leftSpeedSetpoint = targetWheelSpeed.leftMetersPerSecond;
		var rightSpeedSetpoint = targetWheelSpeed.rightMetersPerSecond;

		double leftSpeed = drivetrain.getLeftEncoderSpeed();
		double rightSpeed = drivetrain.getRightEncoderSpeed();

		double rightOutput = rightSpeedController.calculate(rightSpeed, rightSpeedSetpoint);
		double leftOutput = leftSpeedController.calculate(leftSpeed, leftSpeedSetpoint);

		drivetrain.tankDrive(leftOutput, rightOutput);

		// DifferentialDriveKinematics sdf = new DifferentialDriveKinematics(0);
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		if(trajectory.getTotalTimeSeconds() == timer.get()){
			return true;
		}
		return false;
		// trajectory.getTotalTimeSeconds()
		// trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getTranslation().getDistance(drivetrain.getPose().getTranslation())
		
	}

	@Override
	public void end(boolean interrupted) {
		// TODO Auto-generated method stub
		super.end(interrupted);
	}

}
