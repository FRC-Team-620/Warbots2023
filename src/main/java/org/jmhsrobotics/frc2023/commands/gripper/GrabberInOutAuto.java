package org.jmhsrobotics.frc2023.commands.gripper;

import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberInOutAuto extends CommandBase {
	IntakeSubsystem intakeSubsystem;
	boolean intakeIn;
	private double speed;
	private boolean state;
	Timer time = new Timer();

	public GrabberInOutAuto(IntakeSubsystem intakeSubsystem, boolean intakeIn, double speed) {
		this.intakeSubsystem = intakeSubsystem;
		this.intakeIn = intakeIn;
		this.speed = speed;
	}

	@Override
	public void initialize() {
		this.intakeSubsystem.setIntakeMotor(speed * (intakeIn ? -1 : 1));
		// this.intakeSubsystem.switchIntakePistonState(state);

	}
	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			this.intakeSubsystem.stopIntakeMotor();;
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
