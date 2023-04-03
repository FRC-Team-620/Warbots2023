package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Command the Extension and then command the Arm pitch */
public class CommandEThenA extends SequentialCommandGroup {

	/** Command the Extension and then command the Arm pitch */
	public CommandEThenA(ArmSubsystem armSubsystem, double extensionProportion, double angle,
			BooleanSupplier interrupt) {

		// spotless:off
		this.addCommands(
			new CommandArmExtension(armSubsystem, extensionProportion, interrupt),
			new CommandArmPitch(armSubsystem, angle, interrupt)
		);
		// spotless:on
	}
}
