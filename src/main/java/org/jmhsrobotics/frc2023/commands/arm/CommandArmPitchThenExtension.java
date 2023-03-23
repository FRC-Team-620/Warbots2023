package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandArmPitchThenExtension extends SequentialCommandGroup {

	public CommandArmPitchThenExtension(ArmSubsystem armSubsystem, double extensionProportion, double angle,
			BooleanSupplier interrupt) {
		this.addCommands(new CommandArmPitch(armSubsystem, angle, interrupt),
				new CommandArmExtension(armSubsystem, extensionProportion, interrupt));
	}
}
