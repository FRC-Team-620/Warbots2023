package org.jmhsrobotics.frc2023.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.Constants.ArmConstants;
import org.jmhsrobotics.frc2023.Constants.RobotConstants;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TelopArmOpenLoop extends CommandBase {

	ArmSubsystem armSubsystem;
	Supplier<Double> pitchSpeed, linearSpeed;
	BooleanSupplier overrideLimits;

	boolean wasEnded = false;
	double desiredPitch;
	double desiredExtension;

	double pitchFactor;
	double extensionFactor;

	// spotless:off
	public TelopArmOpenLoop(ArmSubsystem armSubsystem, Supplier<Double> pitchSpeed, 
		Supplier<Double> linearSpeed, BooleanSupplier overrideLimits) {

		this.armSubsystem = armSubsystem;
		this.pitchSpeed = pitchSpeed;
		this.linearSpeed = linearSpeed;
		this.overrideLimits = overrideLimits;

		this.pitchFactor = -1 * armSubsystem.getMaxPitchPPIDVel() * RobotConstants.secondsPerTick;
		this.extensionFactor = armSubsystem.getMaxExtensionPPIDVel() * RobotConstants.secondsPerTick;

		addRequirements(armSubsystem);
	}
	// spotless:on

	private void resetDesiredStateToCurrent() {
		// reset desired pitch values
		this.desiredPitch = this.armSubsystem.getArmPitch();
		this.desiredExtension = this.armSubsystem.getArmLength();
		// reset PPIDs (profiled PIDs)
		armSubsystem.resetAnglePPIDToCurrent();
		armSubsystem.resetExtensionPPIDToCurrent();
		// reset fatal error flags
		this.armSubsystem.resetFatalExtensionErrorFlag();
		this.armSubsystem.resetFatalPitchErrorFlag();
	}

	@Override
	public void initialize() {
		this.resetDesiredStateToCurrent();
	}

	@Override
	public void execute() {

		SmartDashboard.putNumber("ArmTeleop/Pitch/pitchEncoder", this.armSubsystem.getPitchRelativeEncoderPosition());
		SmartDashboard.putNumber("ArmTeleop/Pitch/pitchInput", this.pitchSpeed.get());
		SmartDashboard.putNumber("ArmTeleop/Pitch/desiredPitch", this.desiredPitch);
		SmartDashboard.putNumber("ArmTeleop/Pitch/armPitch", this.armSubsystem.getArmPitch());
		SmartDashboard.putNumber("ArmTeleop/Pitch/pitchGoalPos", this.armSubsystem.getPitchPPIDGoal().position);

		SmartDashboard.putNumber("ArmTeleop/Extension/extensionInput", this.linearSpeed.get());
		SmartDashboard.putNumber("ArmTeleop/Extension/desiredExtension", this.desiredExtension);
		SmartDashboard.putNumber("ArmTeleop/Extension/armExtension", this.armSubsystem.getArmLength());
		SmartDashboard.putNumber("ArmTeleop/Extension/extensionGoalPos",
				this.armSubsystem.getExtensionPPIDGoal().position);

		// spotless:off
		if(this.overrideLimits.getAsBoolean()) {
			armSubsystem.setDutyCycle(pitchSpeed.get(), linearSpeed.get());
			this.wasEnded = true;
			return;
		}
		// spotless:on

		// If closed-loop control is starting up again after having been interrupted
		if (this.wasEnded) {
			this.wasEnded = false;
			// resetting the arm to a given min pitch (resetting relative encoder) if below
			// range
			if (this.armSubsystem.getArmPitch() < ArmConstants.minArmAngleDegrees) {
				this.armSubsystem.resetPitchEncoder();
				this.resetDesiredStateToCurrent();
				// do this instead of setting the relative encoder position to 0 (too slow,
				// doesn't work)
				this.armSubsystem.resetAnglePPIDToValue(ArmConstants.stowedDegrees);
			} else { // NORMALLY CALLED
				this.resetDesiredStateToCurrent();
			}
		}

		if (this.armSubsystem.detectedFatalExtensionError() || this.armSubsystem.detectedFatalPitchError()) {
			this.resetDesiredStateToCurrent(); // also resets fatal error flags
		}

		// spotless:off
		double deltaPitch = pitchFactor * pitchSpeed.get();
		this.desiredPitch = MathUtil.clamp(
			this.desiredPitch + deltaPitch, 
			ArmConstants.minArmAngleDegrees,
			ArmConstants.maxArmAngleDegrees
		);

		double deltaExtension = extensionFactor * linearSpeed.get();
		this.desiredExtension = MathUtil.clamp(
			this.desiredExtension + deltaExtension, 
			ArmConstants.minExtensionLengthMillims, 
			ArmConstants.maxExtensionLengthMillims
		);
		// spotless:on

		armSubsystem.updatePitchGoal(this.desiredPitch);
		armSubsystem.updateExtensionGoal(this.desiredExtension);
	}

	@Override
	public void end(boolean interrupted) {
		this.wasEnded = true;
		armSubsystem.stop();
	}
}
