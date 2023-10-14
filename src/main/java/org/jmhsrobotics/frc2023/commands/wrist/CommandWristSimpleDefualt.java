package org.jmhsrobotics.frc2023.commands.wrist;

import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandWristSimpleDefualt extends CommandBase {
	WristSubsystem wristSubsystem;
	ProfiledPIDController wristPID;
	Constraints wristConstraint;

	public CommandWristSimpleDefualt(WristSubsystem wristSubsystem) {
		this.wristSubsystem = wristSubsystem;
		this.wristConstraint = new Constraints(0.9, 0.8);
		this.wristPID = new ProfiledPIDController(4, 0, 0, this.wristConstraint);
		// SmartDashboard.putData("WristPID", this.wristPID);
		addRequirements(this.wristSubsystem);
	}

	@Override
	public void initialize() {
		this.wristPID.reset(new State(this.wristSubsystem.getWristPitch(), 0));
		this.wristPID.setGoal(this.wristSubsystem.getWristPitch());
		this.wristPID.setTolerance(0.1, 0.2);
	}
	@Override
	public void execute() {

		// calculate motor output from pid controller
		double motorRawOutput = this.wristPID.calculate(this.wristSubsystem.getWristPitch());

		double motorOutput = MathUtil.clamp(motorRawOutput, -0.6, 0.6);

		// pass motor output to motor in subsystemw
		this.wristSubsystem.setWristMotor(motorOutput);

	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
