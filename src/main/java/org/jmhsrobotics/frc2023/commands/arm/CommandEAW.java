package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.commands.wrist.CommandWristAbsolute;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** Command the Extension, Arm pitch, and (absolute) Wrist pitch at once */
public class CommandEAW extends ParallelCommandGroup {

	/** Command the Extension, Arm pitch, and (absolute) Wrist pitch at once */
	public CommandEAW(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, double distanceProportion,
			double armAngle, double wristAngle, BooleanSupplier override) {

		// spotless:off
		this.addCommands(
			new CommandArmExtension(armSubsystem, distanceProportion, override),
			new CommandArmPitch(armSubsystem, armAngle, override),
			new CommandWristAbsolute(wristSubsystem, wristAngle, () -> armAngle, override)
        );
        // spotless:on
	}
}
