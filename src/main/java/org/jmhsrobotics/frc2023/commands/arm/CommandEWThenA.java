package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.commands.wrist.CommandWristAbsolute;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Command the Extension and (absolute) Wrist pitch at once and then command the
 * Arm pitch
 */
public class CommandEWThenA extends SequentialCommandGroup {

	/**
	 * Command the Extension and (absolute) Wrist pitch at once and then command the
	 * Arm pitch
	 */
	public CommandEWThenA(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, double distanceProportion,
			double armAngle, double wristAngle, BooleanSupplier override) {

		// spotless:off
		this.addCommands(
            new ParallelCommandGroup(
                new CommandArmExtension(armSubsystem, distanceProportion, override),
                new CommandWristAbsolute(wristSubsystem, wristAngle, () -> armAngle, override)
            ),
            new CommandArmPitch(armSubsystem, armAngle, override)
        );
        // spotless:on
	}

}
