package org.jmhsrobotics.frc2023.commands;

import org.jmhsrobotics.frc2023.subsystems.GrabberMotorSubsystem;
import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberInOutAuto extends CommandBase {
	boolean intakeIn;
	private double speed;
	private boolean state;
	Timer time = new Timer();
	GrabberMotorSubsystem grabberMotorSubsystem;
	GrabberSolenoidSubsystem grabberSolenoidSubsystem;

	public GrabberInOutAuto(GrabberMotorSubsystem grabberMotorSubsystem,
			GrabberSolenoidSubsystem grabberSolenoidSubsystem, boolean intakeIn, double speed) {
		this.intakeIn = intakeIn;
		this.speed = speed;
		this.grabberMotorSubsystem = grabberMotorSubsystem;
		this.grabberSolenoidSubsystem = grabberSolenoidSubsystem;
	}

	@Override
	public void initialize() {
		grabberMotorSubsystem.setGrabberMotor(speed * (intakeIn ? -1 : 1));
		grabberSolenoidSubsystem.setGrabberIntakeState(state);

	}
	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			grabberMotorSubsystem.setGrabberMotor(0);
		}
	}

	// isFinished
	@Override
	public boolean isFinished() {
		if (time.hasElapsed(1)) {
			return true;
		}
		return false;
	}
}
