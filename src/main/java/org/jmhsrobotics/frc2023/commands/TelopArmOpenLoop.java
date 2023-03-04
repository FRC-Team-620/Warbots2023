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
			armSubsystem.setDutyCycle(
				MathUtil.applyDeadband(pitchSpeed.get(), 0.2),
				MathUtil.applyDeadband(linearSpeed.get(), 0.2)
			);
			this.wasEnded = true;
			return;
		}
		// spotless:on

		if (this.wasEnded) {
			this.initialize();
			this.wasEnded = false;
			this.armSubsystem.resetAnglePPIDToCurrent();
			this.armSubsystem.resetExtensionPPIDToCurrent();
			// TODO: Fix this: sets the encoder position after the pid loops have been reset
			// resulting in the loops
			// trying to drive back the negitive delta between the old and reset position of
			// zero.
			// Also Zero degrees is poiting directly inside of the ground. (Prob not a great
			// value to reset to.)
			// if (this.armSubsystem.armPitchDegrees() < ArmConstants.stowedDegrees) {
			// this.armSubsystem.resetPitchEncoder();
			// }
		}

		// spotless:off
		double deltaPitch = pitchFactor * MathUtil.applyDeadband(pitchSpeed.get(), 0.2);
		this.desiredPitch = MathUtil.clamp(
			this.desiredPitch + deltaPitch, 
			ArmConstants.minArmAngleDegrees,
			ArmConstants.maxArmAngleDegrees
		);

		double deltaExtension = extensionFactor * MathUtil.applyDeadband(linearSpeed.get(), 0.2);
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
