package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.commands.wrist.CommandWristAbsolute;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/** Command the Extension, Arm pitch, and (absolute) Wrist pitch */
public class CommandEAW extends ParallelCommandGroup {

	public CommandEAW(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, double distanceProportion,
			double armAngle, double wristAngle, BooleanSupplier override) {

		// spotless:off
		this.addCommands(
            new CommandArm(armSubsystem, distanceProportion, armAngle, override),
			new CommandWristAbsolute(wristSubsystem, wristAngle, armSubsystem::getArmPitch, override)
        );
        // spotless:on
	}
}
