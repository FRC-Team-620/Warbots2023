package org.jmhsrobotics.frc2023.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.Constants.ArmConstants;
import org.jmhsrobotics.frc2023.Constants.RobotConstants;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

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
		this.desiredPitch = this.armSubsystem.getArmPitch();
		this.desiredExtension = this.armSubsystem.getArmLength();
		armSubsystem.resetAnglePPIDToCurrent();
		armSubsystem.resetExtensionPPIDToCurrent();
	}

	@Override
	public void initialize() {
		this.resetDesiredStateToCurrent();
	}

	@Override
	public void execute() {

		SmartDashboard.putNumber("ArmTeleop/Pitch/pitchEncoder", this.armSubsystem.getPitchAbsEncoderPosition());
		SmartDashboard.putNumber("ArmTeleop/Pitch/pitchInput", this.pitchSpeed.get());
		SmartDashboard.putNumber("ArmTeleop/Pitch/desiredPitch", this.desiredPitch);
		SmartDashboard.putNumber("ArmTeleop/Pitch/armPitch", this.armSubsystem.getArmPitch());
		SmartDashboard.putNumber("ArmTeleop/Pitch/pitchGoalPos", this.armSubsystem.getPitchPPIDGoal().position);

		SmartDashboard.putNumber("ArmTeleop/Extension/extensionInput", this.linearSpeed.get());
		SmartDashboard.putNumber("ArmTeleop/Extension/desiredExtension", this.desiredExtension);
		SmartDashboard.putNumber("ArmTeleop/Extension/armExtension", this.armSubsystem.getArmLength());
		SmartDashboard.putNumber("ArmTeleop/Extension/extensionGoalPos",
				this.armSubsystem.getExtensionPPIDGoal().position);

		// For the LEDs
		this.armSubsystem.setTeleopWasEnded(this.wasEnded);

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
			boolean extensionTooLow = this.armSubsystem.getArmLength() < ArmConstants.minExtensionLengthMillims;
			// resetting the arm to a given min pitch (resetting relative encoder) if below
			// range
			if (extensionTooLow) {
				this.armSubsystem.resetExtensionEncoder();
				this.resetDesiredStateToCurrent();
				// vvvvv YOU NEED TO DO THIS vvvvvv
				// RESETTING THE PITCH ENCODER ALONE IS NOT FAST ENOUGH
				// THE HARDWARE IS BAD AND IT DOES NOT REGISTER IN TIME (caused massive jiggle
				// w/o)
				// this killed me
				this.armSubsystem.resetExtensionPPIDToValue(0.0);
			} else {
				this.resetDesiredStateToCurrent();
			}
		}

		// spotless:off
		double deltaPitch = pitchFactor * pitchSpeed.get();
		this.desiredPitch = ArmSubsystem.clampArmPitch(this.desiredPitch + deltaPitch);
		// this.desiredPitch = MathUtil.clamp(
		// 	this.desiredPitch + deltaPitch, 
		// 	ArmConstants.minArmAngleDegrees,
		// 	ArmConstants.maxArmAngleDegrees
		// );

		double deltaExtension = extensionFactor * linearSpeed.get();
		this.desiredExtension = ArmSubsystem.clampExtensionCount(this.desiredExtension + deltaExtension);
		// this.desiredExtension = MathUtil.clamp(
		// 	this.desiredExtension + deltaExtension, 
		// 	ArmConstants.minExtensionLengthMillims, 
		// 	ArmConstants.maxExtensionLengthMillims
		// );
		// spotless:on

		armSubsystem.updatePitchGoal(this.desiredPitch);
		// armSubsystem.setPitch(this.desiredPitch);
		armSubsystem.setExtension(this.desiredExtension);
	}

	@Override
	public void end(boolean interrupted) {
		this.wasEnded = true;
		this.armSubsystem.setTeleopWasEnded(true);
		armSubsystem.stop();
	}
}
