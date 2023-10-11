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
		// this.speed = speed;
		// this.driver = driver;
		this.wristConstraint = new Constraints(0.9, 0.8);
		this.wristPID = new ProfiledPIDController(4, 0, 0, this.wristConstraint);
		this.positionGoal = positionGoal;
		// SmartDashboard.putData("WristPID", this.wristPID);
		addRequirements(wristSubsystem);
	}

	@Override
	public void initialize() {
		this.wristPID.reset(new State(this.positionGoal, 0));
		// wristPID.setGoal(new State(0, 0));
		this.wristPID.setGoal(this.positionGoal);
	}
	@Override
	public void execute() {
		// if ((wristSubsystem.getWristPitch() < 0.8 && wristSubsystem.getWristPitch() >
		// 0.1)) {
		// //
		// this.wristSubsystem.setWristMotor(MathUtil.applyDeadband(driver.getRightY(),
		// 0.1));
		// // System.out.println("mark");

		// } else {
		// this.wristSubsystem.setWristMotor(0);
		// }
		// this.wristSubsystem.setPitch(this.angle.get()); // clamps the input

		// calculate motor output from pid controller
		double motorRawOutput = this.wristPID.calculate(this.wristSubsystem.getWristPitch());

		double motorOutput = MathUtil.clamp(motorRawOutput, -0.6, 0.6);

		// pass motor output to motor in subsystemw
		this.wristSubsystem.setWristMotor(motorOutput);

		SmartDashboard.putNumber("WristPID/output", motorOutput);
		SmartDashboard.putNumber("WristPID/setpoint", this.wristPID.getSetpoint().position);
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
