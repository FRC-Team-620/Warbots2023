package org.jmhsrobotics.frc2023.commands.wrist;

import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandSetGripperOpen extends InstantCommand {
	private IntakeSubsystem intake;
	private boolean open;
	public CommandSetGripperOpen(IntakeSubsystem intake, boolean open) {
		this.intake = intake;
		this.open = open;
		addRequirements(this.intake);

	}
	@Override
	public void execute() {
		this.intake.setIntakePistonState(this.open);
	}

}
