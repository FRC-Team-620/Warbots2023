package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.jmhsrobotics.frc2023.commands.AutoDriveDistance;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;

public class CenterChargeStationAuto extends SequentialCommandGroup {

	Drivetrain drivetrain;
	LEDSubsystem ledSubsystem;

	// Constructor
	public CenterChargeStationAuto(Drivetrain drivetrain, LEDSubsystem ledSubsystem) {
		this.drivetrain = drivetrain;
		this.ledSubsystem = ledSubsystem;

		addCommands(new InstantCommand(() -> { // Set starting location of the robot
			drivetrain.resetOdometry(
					new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079), Rotation2d.fromDegrees(180)));
		}), new AutoDriveDistance(drivetrain, -4.5),
				// Gets the robot out the comunity area (Over the charge station) by driving
				// backwards
				new InstantCommand(() -> {
					LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
				}), new WaitCommand(0.5), // gets robot to pause outside of community
				new AutoBalance(drivetrain, true, ledSubsystem));

	}
}
