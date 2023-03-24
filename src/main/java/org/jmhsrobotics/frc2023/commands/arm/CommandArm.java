package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.Constants.ControlMode;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArm extends CommandBase {

	ArmSubsystem armSubsystem;
	double distanceProportion;
	double angle;
	BooleanSupplier interrupt;

	// Constructor
	// Takes in a angle for the arm pitch and a distance for the linear joint
	// (Prizmatic joint)
	public CommandArm(ArmSubsystem armSubsystem, double distanceProportion, double angle, BooleanSupplier interrupt) {
		this.armSubsystem = armSubsystem;
		this.distanceProportion = MathUtil.clamp(distanceProportion, 0.0, 1.0);
		this.angle = angle;
		this.interrupt = interrupt;
		addRequirements(armSubsystem);
	}

	// execute
	@Override
	public void initialize() {
		this.armSubsystem.setControlMode(ControlMode.CLOSED_LOOP);
		armSubsystem.setExtensionProportion(distanceProportion);
		armSubsystem.setPitch(angle);
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			armSubsystem.stop();
		}
	}

	// isFinished
	@Override
	public boolean isFinished() {
		return (armSubsystem.atPitchGoal() && armSubsystem.atExtensionGoal()) || this.interrupt.getAsBoolean();
	}
}
