package org.jmhsrobotics.frc2023.commands.arm;

import org.jmhsrobotics.frc2023.subsystems.ArmPitchSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArmPitchSimpleDefault extends CommandBase {
	ArmPitchSubsystem pitchSubsystem;
	ProfiledPIDController pitchPID;
	Constraints pitchConstraint;

	public CommandArmPitchSimpleDefault(ArmPitchSubsystem pitchSubsystem) {
		this.pitchSubsystem = pitchSubsystem;
		this.pitchConstraint = new Constraints(100, 120);
		this.pitchPID = new ProfiledPIDController(0.1, 0, 0, this.pitchConstraint);
		// SmartDashboard.putData("pitchPID", this.pitchPID);
		addRequirements(this.pitchSubsystem);
	}

	@Override
	public void initialize() {
		this.pitchPID.reset(new State(this.pitchSubsystem.getEncoderPostition(), 0));
		this.pitchPID.setGoal(this.pitchSubsystem.getEncoderPostition());
		this.pitchPID.setTolerance(1.5, 3);
	}

	@Override
	public void execute() {
		// calculate motor output from pid controller
		// uppper limit: -130
		this.pitchPID.setConstraints(new Constraints(100, 120));
		double motorRawOutput = this.pitchPID.calculate(this.pitchSubsystem.getEncoderPostition());
		double powerLim = 0.8;
		double motorOutput = MathUtil.clamp(motorRawOutput, -powerLim, powerLim);

		// pass motor output to motor in subsystem
		this.pitchSubsystem.setSpeed(motorOutput);
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	// TODO: We should add an end method that sets the motor output to 0 when the
	// command ends;

}
