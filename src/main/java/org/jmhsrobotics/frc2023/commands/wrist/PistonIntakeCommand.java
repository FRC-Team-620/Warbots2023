package org.jmhsrobotics.frc2023.commands.wrist;

import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PistonIntakeCommand extends InstantCommand {
	private IntakeSubsystem intake;
	private boolean state;

	public PistonIntakeCommand(IntakeSubsystem intake) {
		this.intake = intake;
		// this.state = state = false;
		addRequirements(this.intake);
	}

	@Override
	public void execute() {
		this.intake.switchIntakePistonState();
	}
}
