package org.jmhsrobotics.frc2023.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.Constants.RobotConstants;
import org.jmhsrobotics.frc2023.Constants.WristConstants;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopWristOpenLoop extends CommandBase {

	WristSubsystem wristSubsystem;
	Supplier<Double> armAngle, pitchSpeed;
	BooleanSupplier overrideLimits;

	boolean wasEnded = false;
	double desiredPitch;
	double pitchFactor;

	// spotless:off
	public TeleopWristOpenLoop(WristSubsystem wristSubsystem, Supplier<Double> armAngle, Supplier<Double> pitchSpeed, BooleanSupplier overrideLimits) {

        this.wristSubsystem = wristSubsystem;
		this.armAngle = armAngle;
		this.pitchSpeed = pitchSpeed;
		this.overrideLimits = overrideLimits;

		this.pitchFactor = this.wristSubsystem.getWristPPIDConstraints().maxVelocity * RobotConstants.secondsPerTick;

		addRequirements(wristSubsystem);
	}
	// spotless:on

	private void resetDesiredStateToCurrent() {
		this.desiredPitch = this.wristSubsystem.getWristPitch();
		this.wristSubsystem.resetWristPPIDToCurrent();
	}

	@Override
	public void initialize() {
		this.resetDesiredStateToCurrent();
	}

	@Override
	public void execute() {

		SmartDashboard.putNumber("TeleopWrist/desired pitch", this.desiredPitch);
		SmartDashboard.putNumber("TeleopWrist/wrist pitch", this.wristSubsystem.getWristPitch());

		// spotless:off
		if(this.overrideLimits.getAsBoolean()) {
			this.wristSubsystem.setDutyCycle(this.pitchSpeed.get());
			this.wasEnded = true;
			return;
		}
		// spotless:on

		// If closed-loop control is starting up again after having been interrupted
		if (this.wasEnded) {
			this.wasEnded = false;
			this.resetDesiredStateToCurrent();
		}

		// spotless:off
		double deltaPitch = pitchFactor * pitchSpeed.get();
		this.desiredPitch = MathUtil.clamp(
			this.desiredPitch + deltaPitch, 
			WristConstants.minPitchDegrees,
			WristConstants.maxPitchDegrees
		);
		// spotless:on

		this.wristSubsystem.setPitch(this.desiredPitch);
	}

	@Override
	public void end(boolean interrupted) {
		this.wasEnded = true;
		this.wristSubsystem.stop();
	}
}
