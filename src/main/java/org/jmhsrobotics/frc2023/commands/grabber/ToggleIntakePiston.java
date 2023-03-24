package org.jmhsrobotics.frc2023.commands.grabber;

import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleIntakePiston extends InstantCommand {

	public ToggleIntakePiston(IntakeSubsystem intakeSubsystem) {
		super(intakeSubsystem::toggleIntakePistonState, intakeSubsystem);
	}
}
