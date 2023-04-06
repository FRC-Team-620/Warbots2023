package org.jmhsrobotics.frc2023.commands.arm;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.commands.wrist.CommandWristAbsolute;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandAWThenE extends SequentialCommandGroup {

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
	public CommandAWThenE(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, double distanceProportion,
			Supplier<Double> armAngle, Supplier<Double> wristAngle, BooleanSupplier override) {

		// spotless:off
		this.addCommands(
            new ParallelCommandGroup(
                new CommandArmPitch(armSubsystem, armAngle.get(), override),
                new CommandWristAbsolute(wristSubsystem, wristAngle.get(), () -> armAngle.get(), override)
            ),
            new CommandArmExtension(armSubsystem, distanceProportion, override)
        );
        // spotless:on
	}

}
