package org.jmhsrobotics.frc2023.commands.wrist;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.Constants.ControlMode;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandWristRelative extends CommandBase {

	WristSubsystem wristSubsystem;
	Supplier<Double> angle;
	BooleanSupplier interrupt;

	// Constructor
	public CommandWristRelative(WristSubsystem wristSubsystem, Supplier<Double> angle, BooleanSupplier interrupt) {
		this.wristSubsystem = wristSubsystem;
		this.angle = angle;
		this.interrupt = interrupt;
		addRequirements(wristSubsystem);
	}

	public CommandWristRelative(WristSubsystem wristSubsystem, double angle, BooleanSupplier interrupt) {
		this(wristSubsystem, () -> angle, interrupt);
	}

	// execute
	@Override
	public void initialize() {
		this.wristSubsystem.setControlMode(ControlMode.CLOSED_LOOP);
		// this.wristSubsystem.setPitch(this.angle.get()); // clamps the input
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			this.wristSubsystem.stop();
		}
	}

	// isFinished
	@Override
	public boolean isFinished() {
		// return this.wristSubsystem.atPitchGoal() || this.interrupt.getAsBoolean();
		return false;
	}
}
