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
	double desiredPitch = ArmConstants.stowedDegrees;
	double desiredExtension = 0;

	double pitchFactor;
	double extensionFactor;

	public TelopArmOpenLoop(ArmSubsystem armSubsystem, Supplier<Double> pitchSpeed, Supplier<Double> linearSpeed,
			BooleanSupplier overrideLimits) {
		this.armSubsystem = armSubsystem;
		this.pitchSpeed = pitchSpeed;
		this.linearSpeed = linearSpeed;
		this.overrideLimits = overrideLimits;

		this.pitchFactor = -1 * armSubsystem.getMaxPitchPPIDVel() * RobotConstants.secondsPerTick;
		this.extensionFactor = armSubsystem.getMaxExtensionPPIDVel() * RobotConstants.secondsPerTick;

		addRequirements(armSubsystem);
	}

	@Override
	public void initialize() {
		this.desiredPitch = this.armSubsystem.armPitchDegrees();
		this.desiredExtension = this.armSubsystem.getArmLength();
	}

	@Override
	public void execute() {
		// armSubsystem.setDutyCycle(MathUtil.applyDeadband(pitchSpeed.get(), 0.2),
		// MathUtil.applyDeadband(linearSpeed.get(), 0.2));

		// spotless:off
		if(this.overrideLimits.getAsBoolean()) {
			armSubsystem.setDutyCycle(pitchSpeed.get(), linearSpeed.get());
			this.wasEnded = true;
			return;
		}
		// spotless:on

		if (this.wasEnded) {
			this.wasEnded = false;
			if (this.armSubsystem.armPitchDegrees() < ArmConstants.minArmAngleDegrees) {
				this.armSubsystem.resetPitchEncoder();
			}
			if (this.armSubsystem.getArmLength() < ArmConstants.minExtensionLengthMeters) {
				this.armSubsystem.resetExtensionEncoder();
			}
			this.armSubsystem.resetAnglePPIDToCurrent();
			this.armSubsystem.resetExtensionPPIDToCurrent();
			this.initialize();
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
			ArmConstants.minExtensionLengthMeters, 
			ArmConstants.maxExtensionLengthMeters
		);
		// spotless:on

		// armSubsystem.setPitch(this.desiredPitch);
		// armSubsystem.setExtension(this.desiredExtension);

		armSubsystem.updatePitchGoal(this.desiredPitch);
		armSubsystem.updateExtensionGoal(this.desiredExtension);

		// if (deltaPitch != 0.0) {
		// this.desiredPitch += deltaPitch;
		// armSubsystem.setPitch(this.desiredPitch);
		// }

		// if (deltaExtension != 0.0) {
		// this.desiredExtension += deltaExtension;
		// armSubsystem.setExtension(this.desiredExtension);
		// }

		SmartDashboard.putNumber("ArmTeleop/desiredPitch", this.desiredPitch);
		SmartDashboard.putNumber("ArmTeleop/desiredExtension", this.desiredExtension);
	}

	@Override
	public void end(boolean interrupted) {
		this.wasEnded = true;
		armSubsystem.stop();
	}
}
