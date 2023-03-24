package org.jmhsrobotics.frc2023.commands.grabber;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberInOut extends CommandBase {
	private IntakeSubsystem intakeSubsystem;
	BooleanSupplier intakeIn;
	private double speed;
	private boolean state;
	Timer time = new Timer();

	public GrabberInOut(IntakeSubsystem intakeSubsystem, BooleanSupplier intakeIn, double speed) {
		this.intakeSubsystem = intakeSubsystem;
		this.intakeIn = intakeIn;
		this.speed = speed;

		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		this.intakeSubsystem.setIntakeMotor(speed * (intakeIn.getAsBoolean() ? -1 : 1));
		this.intakeSubsystem.setIntakePistonState(state);

	}
	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			this.intakeSubsystem.stopIntakeMotor();
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
