package org.jmhsrobotics.frc2023.commands;

//import org.apache.commons.io.filefilter.FalseFileFilter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.jmhsrobotics.frc2023.RobotMath;
import org.jmhsrobotics.frc2023.Constants.ArenaConstants;
import org.jmhsrobotics.frc2023.Constants.AutoConstants;
import org.jmhsrobotics.frc2023.RobotMath.DiminishingAverageHandler;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
	private Drivetrain drivetrain;
	private double moveLimit;
	private DiminishingAverageHandler lowPassFilter;
	private PIDController pidController;
	private boolean isBalancing = false;
	private Pose2d initialPosition;
	private Pose2d balancedPosition;
	private boolean atLimit = false;
	private Pose2d limitPosition;

	public AutoBalance(Drivetrain drivetrain, boolean backwards) {
		this.drivetrain = drivetrain;
		lowPassFilter = new DiminishingAverageHandler(0.5);
		pidController = new PIDController(AutoConstants.autoDistanceKP, AutoConstants.autoDistanceKI,
				AutoConstants.autoDistanceKD);
	}
	@Override
	public void initialize() {
		this.initialPosition = this.drivetrain.getPose();
		this.moveLimit = ArenaConstants.kchargeStationLengthMeters / 2;
	}

	@Override
	public void execute() {
		double pitch = lowPassFilter.feed(drivetrain.getPitch());
		if (RobotMath.approximatelyZero(pitch, 8)) {
			if (!this.isBalancing) {
				this.isBalancing = true;
				this.atLimit = false;
				this.balancedPosition = this.drivetrain.getPose();
				this.pidController.reset();
				this.pidController.setSetpoint(0);
			}
			this.drivetrain.setCurvatureDrive(this.pidController.calculate(getRelativeDistance(this.balancedPosition)),
					0, false);

		} else {
			this.isBalancing = false;

			if (pitch < 0 && getRelativeDistance(initialPosition) > moveLimit) {
				this.drivetrain.setCurvatureDrive(-1 * AutoConstants.balanceCreepSpeed, 0, false);
			} else if (pitch > 0 && getRelativeDistance(initialPosition) < moveLimit) {
				this.drivetrain.setCurvatureDrive(AutoConstants.balanceCreepSpeed, 0, false);
			} else {
				if (!atLimit) {
					atLimit = true;
					limitPosition = this.drivetrain.getPose();
					this.pidController.setSetpoint(0);
					this.pidController.reset();
				}
				this.drivetrain.setCurvatureDrive(this.pidController.calculate(getRelativeDistance(limitPosition)), 0,
						false);
			}

		}
	}

	private double getRelativeDistance(Pose2d position) {
		return new Transform2d(position, this.drivetrain.getPose()).getTranslation().getX();
	}
	@Override
	public boolean isFinished() {
		return false;
	}

}
