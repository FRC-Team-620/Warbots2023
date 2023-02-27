package org.jmhsrobotics.frc2023.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CompControl implements ControlBoard {

	private CommandXboxController driverController;
	private CommandXboxController operatorController;
	private final double deadband;

	public CompControl() {
		driverController = new CommandXboxController(0);
		operatorController = new CommandXboxController(1);
		deadband = 0.1;
	}

	@Override
	public double driveForward() {
		// The if statement allows for the left and right inputs to be pressed down at
		// the same time but the one pressed down more
		// controls the bot
		return driverController.getRightTriggerAxis() > driverController.getLeftTriggerAxis()
				? driverController.getRightTriggerAxis()
				: -driverController.getLeftTriggerAxis();
	}

	@Override
	public boolean isDriveSlow() {
		return driverController.rightBumper().getAsBoolean();
	}

	@Override
	public double driveTurn() {
		return MathUtil.applyDeadband(driverController.getRightX(), deadband);
	}

	@Override
	public boolean isQuickTurn() {
		return !driverController.a().getAsBoolean();
	}

	@Override
	public double armPitch() {
		return MathUtil.applyDeadband(operatorController.getLeftY(), deadband);
	}

	@Override
	public double armExtend() {
		return MathUtil.applyDeadband(operatorController.getLeftX(), deadband);
	}

	@Override
	public Trigger armPresetFloor() {
		return operatorController.povDown();
	}

	@Override
	public Trigger armPresetMid() {
		return operatorController.povLeft();
	}

	@Override
	public Trigger armPresetHigh() {
		return operatorController.povUp();
	}

	@Override
	public Trigger armStop() {
		return operatorController.start();
	}

	@Override
	public Trigger Intake() {
		return operatorController.leftBumper();
	}

	@Override
	public Trigger armWrist() {
		return operatorController.y();
	}

	@Override
	public Trigger alignPeg() {
		return operatorController.x();
	}

}
