package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

public class BaseLineAuto extends SequentialCommandGroup {

	Drivetrain drivetrain;

	// Constructor
	// public BaseLineAuto(Drivetrain drivetrain) {
	// this.drivetrain = drivetrain;

	// addCommands(new InstantCommand(() -> {
	// drivetrain.resetOdometry(
	// new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079),
	// Rotation2d.fromDegrees(180)));
	// }), new AutoDriveDistance(drivetrain, -4.5), new WaitCommand(0.5), new
	// AutoDriveDistance(drivetrain, 3.5));
	// }

}
