package org.jmhsrobotics.frc2023.commands.grabber;

import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandGrabberSolenoids extends InstantCommand {

	public CommandGrabberSolenoids(GrabberSolenoidSubsystem grabberSolenoidSubsystem, boolean intakeState,
			boolean pitchState) {
		super(() -> {
			grabberSolenoidSubsystem.setGrabberIntakeState(intakeState);
			grabberSolenoidSubsystem.setGrabberPitchState(pitchState);
		}, grabberSolenoidSubsystem);
	}
}
