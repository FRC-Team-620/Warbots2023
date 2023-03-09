package org.jmhsrobotics.frc2023.commands;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArm extends CommandBase {

	ArmSubsystem armSubsystem;
	double distance;
	double angle;
	BooleanSupplier interrupt;

	// Constructor
	// Takes in a angle for the arm pitch and a distance for the linear joint
	// (Prizmatic joint)
	public CommandArm(ArmSubsystem armSubsystem, double distance, double angle, BooleanSupplier interrupt) {
		this.armSubsystem = armSubsystem;
		this.distance = distance;
		this.angle = angle;
		this.interrupt = interrupt;
		addRequirements(armSubsystem);
	}

	// execute
	@Override
	public void initialize() {
		this.armSubsystem.setControlMode(ArmSubsystem.ControlMode.CLOSED_LOOP);
		armSubsystem.setExtension(distance);
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
