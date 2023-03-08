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

	@Override
	public void initialize() {
		this.resetDesiredStateToCurrent();
	}

	private void resetDesiredStateToCurrent() {
		this.desiredPitch = this.armSubsystem.getArmPitch();
		this.desiredExtension = this.armSubsystem.getArmLength();
	}

	@Override
	public void execute() {

		SmartDashboard.putNumber("ArmTeleop/desiredPitch", this.desiredPitch);
		SmartDashboard.putNumber("ArmTeleop/desiredExtension", this.desiredExtension);
		SmartDashboard.putNumber("ArmTeleop/armPitch", this.armSubsystem.getArmPitch());
		SmartDashboard.putNumber("ArmTeleop/armExtension", this.armSubsystem.getArmLength());
		SmartDashboard.putNumber("ArmTeleop/pitchGoalPos", this.armSubsystem.getPitchPPIDGoal().position);
		SmartDashboard.putNumber("ArmTeleop/extensionGoalPos", this.armSubsystem.getExtensionPPIDGoal().position);
		SmartDashboard.putNumber("ArmTeleop/pitchEncoder", this.armSubsystem.getPitchRelativeEncoderPosition());
		SmartDashboard.putNumber("ArmTeleop/extensionEncoder", this.armSubsystem.getExtensionRelativeEncoderPosition());

		// spotless:off
		if(this.overrideLimits.getAsBoolean()) {
			armSubsystem.setDutyCycle(pitchSpeed.get(), linearSpeed.get());
			this.wasEnded = true;
			return;
		}
		// spotless:on

		if (this.wasEnded) {
			this.wasEnded = false;
			if (this.armSubsystem.getArmPitch() < ArmConstants.minArmAngleDegrees) {
				this.armSubsystem.resetPitchEncoder();
			}
			// if (this.armSubsystem.getArmLength() <
			// ArmConstants.minExtensionLengthMillims) {
			// this.armSubsystem.resetExtensionEncoder();
			// }
			this.resetDesiredStateToCurrent();
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
