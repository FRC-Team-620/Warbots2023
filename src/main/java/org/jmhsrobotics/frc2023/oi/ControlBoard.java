package org.jmhsrobotics.frc2023.oi;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {
	public double driveForward();
	public boolean isDriveFast();
	public boolean isDriveSlow();
	public Trigger toggleHeadingLock();
	public double driveTurn();
	public boolean isQuickTurn();
	public double armPitch();
	public double wristPitch();
	public double armExtend();
	public double intakeWheels();
	public Trigger changeScoringType();
	public Trigger cubePickUp();
	public Trigger armPresetSlide();
	public Trigger armPresetStowed();
	public Trigger armPresetPickup();
	public Trigger armPresetFloor();
	public Trigger armPresetMid();
	public Trigger armPresetHigh();
	public Trigger armStop();
	public Trigger autoBalance();
	public Trigger override();
	public Trigger wristControlModifier();
	// public double Intake();
	public Trigger Intake();
	public Trigger armWrist();
	public Trigger alignPeg();
	public Trigger switchGrabber();

	public BooleanSupplier overrideTeleopArm();
	public BooleanSupplier overrideTeleopWrist();

}
