package org.jmhsrobotics.frc2023.commands.wrist;

import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandWristSimple extends CommandBase {
	WristSubsystem wristSubsystem;
	ProfiledPIDController wristPID;
	Constraints wristConstraint;
	// in encodercount
	double positionGoal;

	public CommandWristSimple(WristSubsystem wristSubsystem, double positionGoal) {
		this.wristSubsystem = wristSubsystem;
		this.wristConstraint = new Constraints(0.9, 0.8);
		this.wristPID = new ProfiledPIDController(4, 0, 0, this.wristConstraint);
		this.positionGoal = positionGoal;
		addRequirements(wristSubsystem);
	}

	@Override
	public void initialize() {
		this.wristPID.reset(new State(this.positionGoal, 0));
		// wristPID.setGoal(new State(0, 0));
		this.wristPID.setGoal(this.positionGoal);
		this.wristPID.setTolerance(0.05, 0.1);
	}
	@Override
	public void execute() {

		// calculate motor output from pid controller
		double motorRawOutput = this.wristPID.calculate(this.wristSubsystem.getWristPitch());

		double motorOutput = MathUtil.clamp(motorRawOutput, -0.6, 0.6);

		// pass motor output to motor in subsystemw
		this.wristSubsystem.setWristMotor(motorOutput);

		SmartDashboard.putNumber("WristPID/positionError", this.wristPID.getPositionError());
		SmartDashboard.putNumber("WristPID/velocityError", this.wristPID.getVelocityError());
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return this.wristPID.atGoal();
	}

}
