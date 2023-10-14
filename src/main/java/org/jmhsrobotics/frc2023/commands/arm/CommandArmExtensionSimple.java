package org.jmhsrobotics.frc2023.commands.arm;

import org.jmhsrobotics.frc2023.subsystems.ArmExtensionSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandArmExtensionSimple extends CommandBase {
	ArmExtensionSubsystem extensionSubsystem;
	ProfiledPIDController extensionPID;
	Constraints extensionConstraint;
	// in encodercount
	double positionGoal;

	public CommandArmExtensionSimple(ArmExtensionSubsystem extensionSubsystem, double positionGoal) {
		this.extensionSubsystem = extensionSubsystem;
		this.extensionConstraint = new Constraints(80, 120);
		this.extensionPID = new ProfiledPIDController(0.1, 0, 0, this.extensionConstraint);
		this.positionGoal = positionGoal;
		// SmartDashboard.putData("ExtensionPID", this.extensionPID);
		addRequirements(this.extensionSubsystem);
	}

	@Override
	public void initialize() {
		this.extensionPID.reset(new State(this.positionGoal, 0));
		this.extensionPID.setGoal(this.positionGoal);
		this.extensionPID.setTolerance(2, 3);
	}

	@Override
	public void execute() {
		// calculate motor output from pid controller

		this.extensionPID.setConstraints(new Constraints(80, 120));
		double motorRawOutput = this.extensionPID.calculate(this.extensionSubsystem.getEncoderPostition());
		double powerLim = 0.8;
		double motorOutput = MathUtil.clamp(motorRawOutput, -powerLim, powerLim);

		// pass motor output to motor in subsystem
		this.extensionSubsystem.setSpeed(motorOutput);
		// SmartDashboard.putNumber("ExtensionPID/output", motorOutput);
		// SmartDashboard.putNumber("ExtensionPID/setpoint",
		// this.extensionPID.getSetpoint().position); // 85 is max

		SmartDashboard.putNumber("ExtensionPID/positionError", this.extensionPID.getPositionError());
		SmartDashboard.putNumber("ExtensionPID/velocityError", this.extensionPID.getVelocityError());
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return this.extensionPID.atGoal();
	}

	// TODO: We should add an end method that sets the motor output to 0 when the
	// command ends;

}
