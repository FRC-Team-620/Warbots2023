package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.commands.wrist.CommandWristAbsolute;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Command the Arm pitch and then command the Extension and (absolute) Wrist
 * pitch at once
 */
public class CommandAThenEW extends SequentialCommandGroup {

	/**
	 * Command the Arm pitch and then command the Extension and (absolute) Wrist
	 * pitch at once
	 */
	public CommandAThenEW(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, double distanceProportion,
			double armAngle, double wristAngle, BooleanSupplier override) {

		// spotless:off
		this.addCommands(
            new CommandArmPitch(armSubsystem, armAngle, override),
            new ParallelCommandGroup(
                new CommandArmExtension(armSubsystem, distanceProportion, override),
                new CommandWristAbsolute(wristSubsystem, wristAngle, armSubsystem::getArmPitch, override)
            )
        );
        // spotless:on
	}

}
