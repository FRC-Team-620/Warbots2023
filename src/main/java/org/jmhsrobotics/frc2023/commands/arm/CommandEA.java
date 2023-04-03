package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** Command the Extension and Arm pitch at once */
public class CommandEA extends ParallelCommandGroup {

	/** Command the Extension and Arm pitch at once */
	public CommandEA(ArmSubsystem armSubsystem, double distanceProportion, double angle, BooleanSupplier override) {

		// spotless:off
		this.addCommands(
			new CommandArmExtension(armSubsystem, distanceProportion, override),
			new CommandArmPitch(armSubsystem, angle, override)
		);
		// spotless:on
	}
}
