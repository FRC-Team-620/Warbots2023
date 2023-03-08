package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.jmhsrobotics.frc2023.commands.AutoDriveDistance;
import org.jmhsrobotics.frc2023.commands.CommandArm;
import org.jmhsrobotics.frc2023.commands.GrabberInOutAuto;
import org.jmhsrobotics.frc2023.commands.TurnDeltaAngle;
import org.jmhsrobotics.frc2023.commands.vision.AlignPeg;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.subsystems.GrabberMotorSubsystem;
import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;

public class ScoringAuto extends SequentialCommandGroup {

	Drivetrain drivetrain;
	public enum startingPositions {
		LEFT, CENTER, RIGHT;
	}
	private Enum startingPosition;
	// Constructor
	public ScoringAuto(Drivetrain drivetrain, ArmSubsystem armSubsystem, GrabberMotorSubsystem grabberMotorSubsystem,
			GrabberSolenoidSubsystem grabberSolenoidSubsystem, LEDSubsystem ledSubsystem, Enum startingPosition) {
		this.drivetrain = drivetrain;
		this.startingPosition = startingPosition;
		if (this.startingPosition == startingPositions.LEFT) {
			addCommands(new InstantCommand(() -> {
				drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079),
						Rotation2d.fromDegrees(180)));
			}), new AutoDriveDistance(drivetrain, 2), new TurnDeltaAngle(drivetrain, -90),
					new AutoDriveDistance(drivetrain, 0.2), new TurnDeltaAngle(drivetrain, 90),
					new AutoDriveDistance(drivetrain, 2), new AlignPeg(drivetrain),
					new CommandArm(armSubsystem, 0.2, 90),
					new GrabberInOutAuto(grabberMotorSubsystem, grabberSolenoidSubsystem, false, 1),
					new AutoDriveDistance(drivetrain, -2), new TurnDeltaAngle(drivetrain, -90),
					new AutoDriveDistance(drivetrain, -0.2), new TurnDeltaAngle(drivetrain, 90),
					new AutoDriveDistance(drivetrain, -2), new AutoDriveDistance(drivetrain, -4.5),
					// Gets the robot out the comunity area (Over the charge station) by driving
					// backwards
					new InstantCommand(() -> {
						LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
					}), new WaitCommand(0.5), // gets robot to pause outside of community
					new AutoDriveDistance(drivetrain, 2), new AutoBalance(drivetrain, true, ledSubsystem));

		} else if (this.startingPosition == startingPositions.CENTER) {
			addCommands(new InstantCommand(() -> {
				drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079),
						Rotation2d.fromDegrees(180)));
			}), new AutoDriveDistance(drivetrain, 2), new TurnDeltaAngle(drivetrain, -90),
					new AutoDriveDistance(drivetrain, 0.2), new TurnDeltaAngle(drivetrain, 90),
					new AutoDriveDistance(drivetrain, 2), new AlignPeg(drivetrain),
					new CommandArm(armSubsystem, 0.2, 90),
					new GrabberInOutAuto(grabberMotorSubsystem, grabberSolenoidSubsystem, false, 1),
					new AutoDriveDistance(drivetrain, -2), new TurnDeltaAngle(drivetrain, -90),
					new AutoDriveDistance(drivetrain, -0.2), new TurnDeltaAngle(drivetrain, 90),
					new AutoDriveDistance(drivetrain, -2), new AutoDriveDistance(drivetrain, -4.5),
					// Gets the robot out the comunity area (Over the charge station) by driving
					// backwards
					new InstantCommand(() -> {
						LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
					}), new WaitCommand(0.5), // gets robot to pause outside of community
					new AutoDriveDistance(drivetrain, 2), new AutoBalance(drivetrain, true, ledSubsystem));
		} else {
			addCommands(new InstantCommand(() -> {
				drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079),
						Rotation2d.fromDegrees(180)));
			}), new AutoDriveDistance(drivetrain, 2), new TurnDeltaAngle(drivetrain, -90),
					new AutoDriveDistance(drivetrain, 0.2), new TurnDeltaAngle(drivetrain, 90),
					new AutoDriveDistance(drivetrain, 2), new AlignPeg(drivetrain),
					new CommandArm(armSubsystem, 0.2, 90),
					new GrabberInOutAuto(grabberMotorSubsystem, grabberSolenoidSubsystem, false, 1),
					new AutoDriveDistance(drivetrain, -2), new TurnDeltaAngle(drivetrain, -90),
					new AutoDriveDistance(drivetrain, -0.2), new TurnDeltaAngle(drivetrain, 90),
					new AutoDriveDistance(drivetrain, -2), new AutoDriveDistance(drivetrain, -4.5),
					// Gets the robot out the comunity area (Over the charge station) by driving
					// backwards
					new InstantCommand(() -> {
						LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
					}), new WaitCommand(0.5), // gets robot to pause outside of community
					new AutoDriveDistance(drivetrain, 2), new AutoBalance(drivetrain, true, ledSubsystem));
		}
	}

}
