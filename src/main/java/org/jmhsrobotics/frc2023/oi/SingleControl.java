package org.jmhsrobotics.frc2023.oi;

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
		return controller.rightBumper().getAsBoolean();
	}

	@Override
	public double driveTurn() {
		return MathUtil.applyDeadband(controller.getRightX(), deadband);
	}

	@Override
	public boolean isQuickTurn() {
		return !controller.a().getAsBoolean();
	}

	@Override
	public double armPitch() {
		return MathUtil.applyDeadband(controller.getLeftY(), deadband);
	}

	@Override
	public double armExtend() {
		return MathUtil.applyDeadband(controller.getLeftX(), deadband);
	}

	@Override
	public Trigger armPresetFloor() {
		return controller.povDown();
	}

	@Override
	public Trigger armPresetMid() {
		return controller.povLeft();
	}

	@Override
	public Trigger armPresetHigh() {
		return controller.povUp();
	}

	@Override
	public Trigger armStop() {
		return controller.start();
	}

	@Override
	public Trigger Intake() {
		return controller.leftBumper();
	}

	@Override
	public Trigger armWrist() {
		return controller.y();
	}

	@Override
	public Trigger alignPeg() {
		return controller.x();
	}

	@Override
	public Trigger armPresetStowed() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Trigger armPresetPickup() {
		// TODO Auto-generated method stub
		return null;

	}

	@Override
	public double intakeWheels() {
		return 0;

	}

	@Override
	public Trigger closeGrabber() {
		return controller.rightBumper();
	}

	@Override
	public Trigger autoBalance() {
		return null;
	}

}
