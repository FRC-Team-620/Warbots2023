package org.jmhsrobotics.frc2023.commands.grabber;

import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleGrabberIntake extends InstantCommand {

	public ToggleGrabberIntake(GrabberSolenoidSubsystem grabberSolenoidSubsystem) {
		super(grabberSolenoidSubsystem::toggleIntake, grabberSolenoidSubsystem);
	}
}
