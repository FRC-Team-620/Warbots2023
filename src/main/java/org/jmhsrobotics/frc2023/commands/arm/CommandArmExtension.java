package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArmExtension extends CommandBase {

	ArmSubsystem armSubsystem;
	double distanceProportion;
	BooleanSupplier interrupt;

	// Constructor
	public CommandArmExtension(ArmSubsystem armSubsystem, double distanceProportion, BooleanSupplier interrupt) {
		this.armSubsystem = armSubsystem;
		this.distanceProportion = distanceProportion;
		this.interrupt = interrupt;
		addRequirements(armSubsystem);
	}

	// execute
	@Override
	public void initialize() {
		this.armSubsystem.setControlMode(ArmSubsystem.ControlMode.CLOSED_LOOP);
		armSubsystem.setExtensionProportion(distanceProportion);
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
