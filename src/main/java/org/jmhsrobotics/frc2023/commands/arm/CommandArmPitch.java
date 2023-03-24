package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.Constants.ControlMode;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArmPitch extends CommandBase {

	ArmSubsystem armSubsystem;
	double angle;
	BooleanSupplier interrupt;

	// Constructor
	public CommandArmPitch(ArmSubsystem armSubsystem, double angle, BooleanSupplier interrupt) {
		this.armSubsystem = armSubsystem;
		this.angle = angle;
		this.interrupt = interrupt;
		addRequirements(armSubsystem);
	}

	// execute
	@Override
	public void initialize() {
		this.armSubsystem.setControlMode(ControlMode.CLOSED_LOOP);
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
		return armSubsystem.atPitchGoal() || this.interrupt.getAsBoolean();
	}
}
