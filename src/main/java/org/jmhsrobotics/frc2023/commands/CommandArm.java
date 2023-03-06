package org.jmhsrobotics.frc2023.commands;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArm extends CommandBase {

	ArmSubsystem armSubsystem;
	double distance;
	double angle;

	// Constructor
	// Takes in a angle for the arm pitch and a distance for the linear joint
	// (Prizmatic joint)
	public CommandArm(ArmSubsystem armSubsystem, double distance, double angle) {
		this.armSubsystem = armSubsystem;
		this.distance = distance;
		this.angle = angle;
		addRequirements(armSubsystem);
	}

	// execute
	@Override
	public void initialize() {
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
		return armSubsystem.atPitchGoal() && armSubsystem.atExtensionGoal();
	}
}
