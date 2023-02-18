package org.jmhsrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.jmhsrobotics.frc2023.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GrabberCommand extends CommandBase {

	GrabberSubsystem grabberSubsystem;
	private CommandXboxController operatorController;
	private boolean releaseB = true;

	// Constructor
	public GrabberCommand(GrabberSubsystem grabberSubsystem, CommandXboxController controller) {
		this.grabberSubsystem = grabberSubsystem;
		this.operatorController = controller;

		addRequirements(grabberSubsystem);
	}

	// execute
	@Override
	public void execute() {

		// enables the controller inputs for arm subsystem
		

		// is B pushed and its not the same press as last tick
		if (operatorController.b().getAsBoolean()) {
		}
		if (operatorController.b().getAsBoolean() && releaseB) {
			grabberSubsystem.setGrabberState(!this.grabberSubsystem.getGrabberState());
			releaseB = false;
		} else if (!operatorController.b().getAsBoolean()) {
			releaseB = true;
		}
		// enables the motor to control it
		if (operatorController.x().getAsBoolean()) {
			grabberSubsystem.wheelForward();
		}
		else if (operatorController.y().getAsBoolean()) {
			grabberSubsystem.wheelBackward();
		}
		else {
			grabberSubsystem.stopGrabberWheel();
		}

	}

	// isFinished
	@Override
	public boolean isFinished() {
		return false;
	}
}
