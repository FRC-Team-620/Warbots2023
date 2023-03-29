package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.commands.wrist.CommandWristAbsolute;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandEThenAW extends SequentialCommandGroup {

	public CommandEThenAW(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, double distanceProportion,
			double armAngle, double wristAngle, BooleanSupplier override) {

		// spotless:off
		this.addCommands(
            new CommandArmExtension(armSubsystem, distanceProportion, override),
            new ParallelCommandGroup(
                new CommandArmPitch(armSubsystem, armAngle, override),
                new CommandWristAbsolute(wristSubsystem, wristAngle, () -> armAngle, override)
            )
        );
        // spotless:on
	}

}
