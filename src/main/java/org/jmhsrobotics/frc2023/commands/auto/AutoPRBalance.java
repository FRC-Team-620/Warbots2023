package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

//import org.jmhsrobotics.frc2023.RobotContainer;
import org.jmhsrobotics.frc2023.Constants.AutoConstants;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

public class AutoPRBalance extends CommandBase {

	private Drivetrain drivetrain;

	// private RobotMath.DiminishingAverageHandler pitchHandler;
	// private RobotMath.DiminishingAverageHandler rollHandler;

	private PIDController pitchSpeedPID;
	private PIDController rollCurvaturePID;

	// private double pitch;
	// private double roll;

	private AutoPRBalance(Drivetrain drivetrain) {

		this.drivetrain = drivetrain;

		// this.pitchHandler = new RobotMath.DiminishingAverageHandler(2.0);
		// this.rollHandler = new RobotMath.DiminishingAverageHandler(2.0);

		// spotless:off
		this.pitchSpeedPID = new PIDController(
            AutoConstants.kPPitchSpeed, 
            AutoConstants.kIPitchSpeed,
			AutoConstants.kDPitchSpeed
        );
        // spotless:on

		this.pitchSpeedPID.setSetpoint(0.0);
		this.pitchSpeedPID.setTolerance(0.3, 40.0);

		// spotless:off
		this.rollCurvaturePID = new PIDController(
            AutoConstants.kPRollCurvature, 
            AutoConstants.kIRollCurvature,
			AutoConstants.kDRollCurvature
        );
        // spotless:on

		this.rollCurvaturePID.setSetpoint(0.0);
		this.rollCurvaturePID.setTolerance(0.3, 40.0);

	}

	@Override
	public void initialize() {

		// this.pitchHandler.reset();
		// this.rollHandler.reset();

		this.pitchSpeedPID.reset();
		this.rollCurvaturePID.reset();

		this.drivetrain.disableHeadingLock();
	}

	// @Override
	// public void execute() {

	// // this.pitch = this.pitchHandler.feed(this.drivetrain.getPitch());
	// // this.roll = this.rollHandler.feed(this.drivetrain.getRoll());

	// IMUState imuState = RobotContainer.getTelemetry().getIMUState();

	// double speed = this.pitchSpeedPID.calculate(imuState.pitch);
	// double curvature = this.rollCurvaturePID.calculate(imuState.roll);

	// this.drivetrain.setCurvatureDrive(speed, curvature, false);
	// }

	// @Override
	// public void end(boolean interrupted) {

	// this.drivetrain.resetHeadingLockPID();

	// this.drivetrain.stop();

	// this.drivetrain.lockCurrentHeading();
	// this.drivetrain.enableHeadingLock();
	// }

	@Override
	public boolean isFinished() {
		return this.pitchSpeedPID.atSetpoint() && this.rollCurvaturePID.atSetpoint();
	}
}
