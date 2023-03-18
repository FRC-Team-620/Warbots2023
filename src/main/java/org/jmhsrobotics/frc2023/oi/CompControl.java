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
	public boolean isDriveSlow() {
		return driverController.rightBumper().getAsBoolean();
	}

	@Override
	public double driveTurn() {
		return MathUtil.applyDeadband(driverController.getLeftX(), deadband);
	}

	@Override
	public boolean isQuickTurn() {
		return !driverController.b().getAsBoolean();
	}

	@Override
	public Trigger alignPeg() {
		return driverController.leftBumper();
	}

	@Override
	public Trigger autoBalance() {
		return driverController.rightStick();
	}

	@Override
	public boolean isDriveFast() {
		// TODO Auto-generated method stub
		return driverController.a().getAsBoolean();
	}

	// Operator Controller

	@Override
	public double armPitch() {
		return MathUtil.applyDeadband(operatorController.getLeftY(), deadband);
	}

	@Override
	public double armExtend() {
		return MathUtil.applyDeadband(-operatorController.getRightY(), deadband);
	}

	public double intakeWheels() {
		return operatorController.getRightTriggerAxis() > operatorController.getLeftTriggerAxis()
				? operatorController.getRightTriggerAxis()
				: -operatorController.getLeftTriggerAxis() * 0.5;

	}

	@Override
	public Trigger armPresetFloor() {
		return operatorController.a();
	}

	@Override
	public Trigger armPresetMid() {
		return operatorController.x();
	}

	@Override
	public Trigger armPresetHigh() {// Not in use right now
		return operatorController.y();
	}

	@Override
	public Trigger armStop() {
		// return operatorController.start();
		return null;
	}

	@Override
	public Trigger overrideTeleopArm() {
		return operatorController.start();
	}

	@Override

	public Trigger Intake() {
		return null;
	}

	@Override
	public Trigger armWrist() {
		return operatorController.povUp();
	}

	@Override
	public Trigger changeScoringType() {
		return operatorController.povRight();
	}

	@Override
	public Trigger armPresetStowed() {
		return operatorController.b();
	}

	@Override
	public Trigger armPresetPickup() {
		return operatorController.povDown();
	}

	@Override
	public Trigger closeGrabber() {
		return operatorController.rightBumper();
	}

	@Override
	public Trigger toggleHeadingLock() {
		return operatorController.back();
	}

	// TODO: delete (WAS TEMPORARY)
	// public CommandXboxController getDriver() {
	// return driverController;
	// }

}
