package org.jmhsrobotics.frc2023.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopWristOpenLoop extends CommandBase {

	WristSubsystem wristSubsystem;
	Supplier<Double> armAngle, pitchSpeed;
	BooleanSupplier wristMod, overrideLimits;

	boolean wasEnded = false;
	/** The desired ABSOLUTE angle of the wrist, NOT relative to the arm angle */
	double desiredPitch;
	double pitchFactor;

	public TeleopWristOpenLoop(WristSubsystem wristSubsystem, Supplier<Double> armAngle, Supplier<Double> pitchSpeed,
			BooleanSupplier wristMod, BooleanSupplier override) {

		this.wristSubsystem = wristSubsystem;
		this.armAngle = armAngle;
		this.pitchSpeed = pitchSpeed;
		this.wristMod = wristMod;
		this.overrideLimits = override;

		this.pitchFactor = 30;

		addRequirements(wristSubsystem);
	}

	private void resetDesiredStateToCurrent() {
		this.desiredPitch = this.wristSubsystem.getWristPitch() + this.armAngle.get();
		this.wristSubsystem.resetWristPPIDToCurrent();
	}

	@Override
	public void initialize() {
		this.resetDesiredStateToCurrent();
	}

	@Override
	public void execute() {

		double currentInput = this.pitchSpeed.get();
		double currentArmPitch = this.armAngle.get();
		double currentWristPitch = this.wristSubsystem.getWristPitch();

		SmartDashboard.putNumber("TeleopWrist/desired pitch", this.desiredPitch);
		SmartDashboard.putNumber("TeleopWrist/arm angle", currentArmPitch);
		SmartDashboard.putNumber("TeleopWrist/wrist pitch", currentWristPitch);
		SmartDashboard.putNumber("TeleopWrist/overridden", this.overrideLimits.getAsBoolean() ? 1 : -1);

		// spotless:off
		if(this.overrideLimits.getAsBoolean()) {
			if(this.wristMod.getAsBoolean())
				this.wristSubsystem.setDutyCycle(currentInput);
			this.wasEnded = true;
			return;
		}
		// spotless:on

		// If closed-loop control is starting up again after having been interrupted
		if (this.wasEnded) {
			this.wasEnded = false;
			this.resetDesiredStateToCurrent();
		}

		// inverse kinematics, kinda
		double gotoPitch = this.desiredPitch - currentArmPitch;

		// will be exactly zero if the wrist-control-modifier button is not pressed
		if (currentInput != 0.0) {
			double deltaPitch = this.pitchFactor * currentInput;
			this.desiredPitch = WristSubsystem.clampWristPitch(currentWristPitch + deltaPitch) + currentArmPitch;
		}

		// automatically clamps it
		this.wristSubsystem.setPitch(gotoPitch);
	}

	@Override
	public void end(boolean interrupted) {
		this.wasEnded = true;
		this.wristSubsystem.stop();
	}
}
