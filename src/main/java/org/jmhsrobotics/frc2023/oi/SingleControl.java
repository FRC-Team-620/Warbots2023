package org.jmhsrobotics.frc2023.oi;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleControl implements ControlBoard {

	private CommandXboxController controller;
	private final double deadband;

	public SingleControl() {
		controller = new CommandXboxController(0);
		deadband = 0.1;
	}

	@Override
	public double driveForward() {
		// The if statement allows for the left and right inputs to be pressed down at
		// the same time but the one pressed down more
		// controls the bot
		return controller.getRightTriggerAxis() > controller.getLeftTriggerAxis()
				? controller.getRightTriggerAxis()
				: -controller.getLeftTriggerAxis();
	}

	@Override
	public boolean isDriveSlow() {
		return true;
	}

	@Override
	public double driveTurn() {
		return MathUtil.applyDeadband(controller.getLeftX(), deadband);
	}

	@Override
	public boolean isQuickTurn() {
		return true;
	}

	private double operatorLeftY() {
		return MathUtil.applyDeadband(controller.getLeftY(), deadband);
	}

	@Override
	public double armPitch() {
		return !this.wristControlModifier().getAsBoolean() ? operatorLeftY() : 0.0;
	}

	@Override
	public double wristPitch() {
		return this.controller.getRightY();
	}

	@Override
	public double armExtend() {
		return MathUtil.applyDeadband(controller.getLeftX(), deadband);
	}

	@Override
	public Trigger armPresetFloor() {
		return null;
	}

	@Override
	public Trigger armPresetMid() {
		return controller.x();
	}

	@Override
	public Trigger armPresetHigh() {
		return null;
	}

	@Override
	public Trigger armStop() {
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
	public Trigger alignPeg() {
		return null;
	}

	@Override
	public Trigger changeScoringType() {
		return null;
	}

	public Trigger armPresetSlide() {
		// TODO Auto-generated method stub
		return controller.povDown();
	}

	@Override
	public Trigger armPresetStowed() {
		// TODO Auto-generated method stub
		return controller.b();
	}

	@Override
	public Trigger armPresetPickup() {
		// TODO Auto-generated method stub
		return null;

	}

	@Override
	public Trigger override() {
		return controller.start();
	}

	@Override
	public Trigger wristControlModifier() {
		return controller.povLeft();
	}

	@Override
	public double intakeWheels() {
		return controller.rightBumper().getAsBoolean() ? 1 : controller.leftBumper().getAsBoolean() ? -1 : 0;

	}

	@Override
	public Trigger switchGrabber() {
		return controller.a();
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

	@Override
	public Trigger toggleHeadingLock() {
		// TODO Auto-generated method stub
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

	@Override
	public Trigger cubePickUp() {
		// TODO Auto-generated method stub
		return null;
	}

}
