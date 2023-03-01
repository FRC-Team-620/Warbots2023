package org.jmhsrobotics.frc2023.commands;

import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TelopArmOpenLoop extends CommandBase {

	ArmSubsystem armSubsystem;
	Supplier<Double> pitchSpeed, linearSpeed;
	public TelopArmOpenLoop(ArmSubsystem armSubsystem, Supplier<Double> pitchSpeed, Supplier<Double> linearSpeed) {
		this.armSubsystem = armSubsystem;
		this.pitchSpeed = pitchSpeed;
		this.linearSpeed = linearSpeed;
		addRequirements(armSubsystem);
	}

	@Override
	public void execute() {
		armSubsystem.setDutyCycle(MathUtil.applyDeadband(pitchSpeed.get(), 0.2),
				MathUtil.applyDeadband(linearSpeed.get(), 0.2));
	}

	@Override
	public void end(boolean interrupted) {
		armSubsystem.stop();
	}
}
