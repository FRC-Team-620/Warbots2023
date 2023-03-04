package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

public class CenterChargeStationAuto extends SequentialCommandGroup {

	Drivetrain drivetrain;

	// Constructor
	// public CenterChargeStationAuto(Drivetrain drivetrain) {
	// this.drivetrain = drivetrain;

	// addCommands(new InstantCommand(() -> { // Set starting location of the robot
	// drivetrain.resetOdometry(
	// new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079),
	// Rotation2d.fromDegrees(180)));
	// }), new AutoDriveDistance(drivetrain, -4.5),
	// // Gets the robot out the comunity area (Over the charge station) by driving
	// // backwards
	// new WaitCommand(0.5), // gets robot to pause outside of community
	// new AutoBalance(drivetrain, false));

	// }
}
