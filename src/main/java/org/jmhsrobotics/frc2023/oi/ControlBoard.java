package org.jmhsrobotics.frc2023.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {
	public double driveForward();
	public boolean isDriveSlow();
	public double driveTurn();
	public boolean isQuickTurn();
	public double armPitch();
	public double armExtend();
	public double intakeWheels();
	public Trigger armPresetStowed();
	public Trigger armPresetPickup();
	public Trigger armPresetFloor();
	public Trigger armPresetMid();
	public Trigger armPresetHigh();
	public Trigger armStop();
	public Trigger autoBalance();
	// public double Intake();
	public Trigger Intake();
	public Trigger armWrist();
	public Trigger alignPeg();
	public Trigger closeGrabber();

}
