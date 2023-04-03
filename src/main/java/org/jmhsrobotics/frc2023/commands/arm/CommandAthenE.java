package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Command the Arm pitch and then command the Extension */
public class CommandAthenE extends SequentialCommandGroup {

	/** Command the Arm pitch and then command the Extension */
	public CommandAthenE(ArmSubsystem armSubsystem, double extensionProportion, double angle,
			BooleanSupplier interrupt) {

		// spotless:off
		this.addCommands(
			new CommandArmPitch(armSubsystem, angle, interrupt),
			new CommandArmExtension(armSubsystem, extensionProportion, interrupt)
		);
		// spotless:on
	}
}
