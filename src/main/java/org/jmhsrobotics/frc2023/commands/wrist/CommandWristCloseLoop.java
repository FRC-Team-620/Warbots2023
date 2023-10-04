package org.jmhsrobotics.frc2023.commands.wrist;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.oi.ControlBoard;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandWristCloseLoop extends CommandBase {
	private WristSubsystem wrist;
	private ControlBoard control;
	private PIDController wristPID;
	private double encoderPerTick;
	private boolean isTooLow;
	private boolean isTooHigh;
	private double currentPitch;
	public CommandWristCloseLoop(WristSubsystem wrist, ControlBoard control) {
		this.wrist = new WristSubsystem();
		this.control = control;
		this.wristPID = new PIDController(0.2, 0, 0);
		this.encoderPerTick = 0.1;
		this.isTooHigh = this.currentPitch < Constants.WristConstants.maxPitchDegrees ? true : false;
		this.isTooLow = this.currentPitch > Constants.WristConstants.minPitchDegrees ? true : false;

		addRequirements(this.wrist);
	}

	private void setSetPoint() {

	}
	@Override
	public void initialize() {
		this.wristPID.setSetpoint(this.wrist.getWristPitch());
	}
	@Override
	public void execute() {
		// getting controller input from
		// control.wristPitch();
		this.wristPID.setSetpoint(control.wristPitch() * this.encoderPerTick);
		currentPitch = this.wrist.getWristPitch();
		double outPut = this.wristPID.calculate(encoderPerTick - currentPitch);
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return super.isFinished();
	}
}
