package org.jmhsrobotics.frc2023.commands.arm;

import org.jmhsrobotics.frc2023.subsystems.ArmPitchSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArmPitchSimple extends CommandBase {
	ArmPitchSubsystem pitchSubsystem;
	ProfiledPIDController pitchPID;
	Constraints pitchConstraint;
	// in encodercount
	double positionGoal;

	public CommandArmPitchSimple(ArmPitchSubsystem pitchSubsystem, double positionGoal) {
		this.pitchSubsystem = pitchSubsystem;
		this.pitchConstraint = new Constraints(100, 120);
		this.pitchPID = new ProfiledPIDController(0.1, 0, 0, this.pitchConstraint);
		this.positionGoal = positionGoal;
		// SmartDashboard.putData("pitchPID", this.pitchPID);
		// SmartDashboard.putNumber("pitchPID/maxVel", this.pitchConstraint.maxVelocity);
		// SmartDashboard.putNumber("pitchPID/maxAccel", this.pitchConstraint.maxAcceleration);
		// SmartDashboard.putNumber("pitchPID/powerLim", 0.8);
		addRequirements(this.pitchSubsystem);
	}

	@Override
	public void initialize() {
		this.pitchPID.reset(new State(this.positionGoal, 0));
		// wristPID.setGoal(new State(0, 0));
		this.pitchPID.setGoal(this.positionGoal);
	}

	@Override
	public void execute() {
		// calculate motor output from pid controller
		// uppper limit: -130
		// double maxVel = SmartDashboard.getNumber("pitchPID/maxVel", this.pitchConstraint.maxVelocity);
		// double maxAccel = SmartDashboard.getNumber("pitchPID/maxAccel", this.pitchConstraint.maxAcceleration);
		this.pitchPID.setConstraints(new Constraints(100, 120));
		double motorRawOutput = this.pitchPID.calculate(this.pitchSubsystem.getEncoderPostition());
		// double powerLim = SmartDashboard.getNumber("pitchPID/powerLim", 0.8);
		double powerLim = 0.8;
		double motorOutput = MathUtil.clamp(motorRawOutput, -powerLim, powerLim);

		// pass motor output to motor in subsystem
		this.pitchSubsystem.setSpeed(motorOutput);
		// SmartDashboard.putNumber("pitchPID/output", motorOutput);
		// SmartDashboard.putNumber("pitchPID/setpoint", this.pitchPID.getSetpoint().position); // 85 is max
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	// TODO: We should add an end method that sets the motor output to 0 when the
	// command ends;

}
