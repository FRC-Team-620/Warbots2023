package org.jmhsrobotics.frc2023.commands.grabber;

import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandIntakeSolenoid extends InstantCommand {

	public CommandIntakeSolenoid(IntakeSubsystem intakeSubsystem, boolean intakeState) {
		super(() -> {
			intakeSubsystem.setIntakePistonState(intakeState);
		}, intakeSubsystem);
	}
}
