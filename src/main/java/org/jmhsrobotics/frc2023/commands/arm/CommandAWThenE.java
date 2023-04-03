package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.commands.wrist.CommandWristAbsolute;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Command the Arm pitch and (absolute) Wrist pitch at once and then command the
 * Extension
 */
public class CommandAWThenE extends SequentialCommandGroup {

	/**
	 * Command the Arm pitch and (absolute) Wrist pitch at once and then command the
	 * Extension
	 */
	public CommandAWThenE(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, double distanceProportion,
			double armAngle, double wristAngle, BooleanSupplier override) {

		// spotless:off
		this.addCommands(
            new ParallelCommandGroup(
                new CommandArmPitch(armSubsystem, armAngle, override),
                new CommandWristAbsolute(wristSubsystem, wristAngle, () -> armAngle, override)
            ),
            new CommandArmExtension(armSubsystem, distanceProportion, override)
        );
        // spotless:on
	}

}
