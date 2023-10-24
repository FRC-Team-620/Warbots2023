package org.jmhsrobotics.frc2023.oi;

import java.util.function.BooleanSupplier;

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

	// Driver Controller
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
	public double driveTurn() {
		return MathUtil.applyDeadband(driverController.getLeftX(), deadband);
	}

	public double intakeWheels() {
		return operatorController.getRightTriggerAxis() > operatorController.getLeftTriggerAxis()
				? -operatorController.getRightTriggerAxis()
				: operatorController.getLeftTriggerAxis();

	}

	@Override
	public Trigger armPresetSlide() {
		return operatorController.povDown();
	}

	@Override
	public Trigger armPresetFloor() {
		return operatorController.a();
	}

	@Override
	public Trigger armPresetStowed() {
		return operatorController.b();
	}

	@Override
	public Trigger armPresetMid() {
		return operatorController.y();
	}

	@Override
	public Trigger switchGrabber() {
		return operatorController.x();
	}

	@Override
	public boolean isDriveSlow() {
		return false;
	}

	@Override
	public boolean isQuickTurn() {
		return true;
	}

	@Override
	public Trigger alignPeg() {
		return null;
	}

	@Override
	public Trigger autoBalance() {
		return null;
	}

	@Override
	public boolean isDriveFast() {
		// TODO Auto-generated method stub
		return false;
	}

	// Operator Controller

	private double operatorLeftY() {
		return 0.0;
	}

	@Override
	public double armPitch() {
		return 0.0;
	}

	@Override
	public double wristPitch() {
		return 0.0;
	}

	@Override
	public double armExtend() {
		return 0.0;
	}

	@Override
	public Trigger armPresetHigh() {// Not in use right now
		return null;
	}

	@Override
	public Trigger armStop() {
		// return operatorController.start();
		return null;
	}

	@Override
	public Trigger override() {
		return null;
	}

	@Override
	public Trigger wristControlModifier() {
		return null;
	}

	@Override
	public Trigger Intake() {
		return null;
	}

	@Override
	public Trigger armWrist() {
		return null;
	}

	@Override
	public Trigger changeScoringType() {
		return null;
	}

	@Override
	public Trigger armPresetPickup() {
		return null;
	}

	@Override
	public Trigger toggleHeadingLock() {
		return null;
	}

	@Override
	public BooleanSupplier overrideTeleopArm() {
		return () -> this.override().getAsBoolean() && !this.wristControlModifier().getAsBoolean();
	}

	@Override
	public BooleanSupplier overrideTeleopWrist() {
		return () -> this.override().getAsBoolean() && this.wristControlModifier().getAsBoolean();
	}

}
