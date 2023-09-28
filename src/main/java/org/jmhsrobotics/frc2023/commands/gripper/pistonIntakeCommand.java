package org.jmhsrobotics.frc2023.commands.gripper;

import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class pistonIntakeCommand extends CommandBase {
	private IntakeSubsystem intake;
	private boolean state;

	public pistonIntakeCommand(IntakeSubsystem intake) {
		this.intake = intake;
		// this.state = state = false;
		addRequirements(this.intake);
	}

	@Override
	public void initialize() {
		boolean pistonOpen = false;
		intake.setIntakePistonState(!pistonOpen);
	}
}
