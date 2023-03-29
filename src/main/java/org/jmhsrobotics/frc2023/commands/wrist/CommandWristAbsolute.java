package org.jmhsrobotics.frc2023.commands.wrist;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

public class CommandWristAbsolute extends CommandWristRelative {

	public CommandWristAbsolute(WristSubsystem wristSubsystem, Supplier<Double> absoluteAngle,
			Supplier<Double> armPitch, BooleanSupplier interrupt) {
		super(wristSubsystem, () -> absoluteAngle.get() - armPitch.get(), interrupt);
	}

	public CommandWristAbsolute(WristSubsystem wristSubsystem, double absoluteAngle, Supplier<Double> armPitch,
			BooleanSupplier interrupt) {
		this(wristSubsystem, () -> absoluteAngle, armPitch, interrupt);
	}
}
