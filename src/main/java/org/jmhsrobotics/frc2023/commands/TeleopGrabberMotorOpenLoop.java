package org.jmhsrobotics.frc2023.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.subsystems.GrabberMotorSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopGrabberMotorOpenLoop extends CommandBase {

	GrabberMotorSubsystem grabberMotorSubsystem;
	Supplier<Double> intakeSpeed;
	BooleanSupplier shouldApplyBaseSpeed, reduceSpeed;

	private final double baseSpeed = 0.05;
	private final double speedReductionFactor = 0.4;

	public TeleopGrabberMotorOpenLoop(GrabberMotorSubsystem grabberMotorSubsystem, Supplier<Double> intakeSpeed,
			BooleanSupplier shouldApplyBaseSpeed, BooleanSupplier reduceSpeed) {

		this.grabberMotorSubsystem = grabberMotorSubsystem;

		this.intakeSpeed = intakeSpeed;
		this.shouldApplyBaseSpeed = shouldApplyBaseSpeed;
		this.reduceSpeed = reduceSpeed;

		addRequirements(grabberMotorSubsystem);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {

		double speed = this.intakeSpeed.get();

		if (this.reduceSpeed.getAsBoolean())
			speed *= this.speedReductionFactor;

		if (this.shouldApplyBaseSpeed.getAsBoolean())
			speed += this.baseSpeed;

		this.grabberMotorSubsystem.setGrabberMotor(MathUtil.clamp(speed, -1.0, 1.0));
	}

	@Override
	public void end(boolean interrupted) {
	}
}
