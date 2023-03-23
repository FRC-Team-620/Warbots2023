package org.jmhsrobotics.frc2023.commands.grabber;

import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleGrabberPitch extends InstantCommand {

	public ToggleGrabberPitch(GrabberSolenoidSubsystem grabberSolenoidSubsystem) {
		super(grabberSolenoidSubsystem::togglePitch, grabberSolenoidSubsystem);
	}
}
