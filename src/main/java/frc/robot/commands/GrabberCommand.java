package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GrabberCommand extends CommandBase {

	GrabberSubsystem grabberSubsystem;
	private CommandXboxController controller;
	private boolean releaseB = true;

	// Constructor
	public GrabberCommand(GrabberSubsystem grabberSubsystem) {
		this.grabberSubsystem = grabberSubsystem;
	}

	// execute
	@Override
	public void execute() {

		// enables the controller inputs for arm subsystem

		// is B pushed and its not the same press as last tick
		if (controller.b().getAsBoolean()) {
		}
		if (controller.b().getAsBoolean() && releaseB) {
			grabberSubsystem.setGrabberState(!this.grabberSubsystem.getGrabberState());
			releaseB = false;
		} else if (!controller.b().getAsBoolean()) {
			releaseB = true;
		}
		// enables the motors to control their respective jobs

	}

	// isFinished
	@Override
	public boolean isFinished() {
		return false;
	}
}
