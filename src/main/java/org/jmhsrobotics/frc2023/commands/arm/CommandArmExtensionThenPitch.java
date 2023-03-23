package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandArmExtensionThenPitch extends SequentialCommandGroup {

	public CommandArmExtensionThenPitch(ArmSubsystem armSubsystem, double extensionProportion, double angle,
			BooleanSupplier interrupt) {
		this.addCommands(new CommandArmExtension(armSubsystem, extensionProportion, interrupt),
				new CommandArmPitch(armSubsystem, angle, interrupt));
	}
}
