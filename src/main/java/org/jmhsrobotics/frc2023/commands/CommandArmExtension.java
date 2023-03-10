package org.jmhsrobotics.frc2023.commands;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArmExtension extends CommandBase {

	ArmSubsystem armSubsystem;
	double distance;
	BooleanSupplier interrupt;

	// Constructor
	public CommandArmExtension(ArmSubsystem armSubsystem, double distance, BooleanSupplier interrupt) {
		this.armSubsystem = armSubsystem;
		this.distance = distance;
		this.interrupt = interrupt;
		addRequirements(armSubsystem);
	}

	// execute
	@Override
	public void initialize() {
		this.armSubsystem.setControlMode(ArmSubsystem.ControlMode.CLOSED_LOOP);
		armSubsystem.setExtension(distance);
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
		return armSubsystem.atExtensionGoal() || this.interrupt.getAsBoolean();
	}
}
