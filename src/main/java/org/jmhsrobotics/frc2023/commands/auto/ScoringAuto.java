package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.jmhsrobotics.frc2023.Constants.ArmConstants;
import org.jmhsrobotics.frc2023.commands.AutoDriveDistance;
import org.jmhsrobotics.frc2023.commands.CommandArm;
import org.jmhsrobotics.frc2023.commands.CommandArmExtension;
import org.jmhsrobotics.frc2023.commands.CommandArmPitch;
import org.jmhsrobotics.frc2023.commands.TurnAngle;
import org.jmhsrobotics.frc2023.oi.ControlBoard;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.subsystems.GrabberMotorSubsystem;
import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.SectionManager;

public class ScoringAuto extends SequentialCommandGroup {

	Drivetrain drivetrain;
	LEDSubsystem ledSubsystem;
	public static enum StartingPosition {
		LEFT, CENTER, RIGHT;
	}
	private StartingPosition startingPos;
	// Constructor
	public ScoringAuto(Drivetrain drivetrain, ArmSubsystem armSubsystem, GrabberMotorSubsystem grabberMotorSubsystem,
			GrabberSolenoidSubsystem grabberSolenoidSubsystem, LEDSubsystem ledSubsystem, StartingPosition startingPos,
			ControlBoard controls) {
		this.drivetrain = drivetrain;
		this.ledSubsystem = ledSubsystem;
		this.startingPos = startingPos;

		addCommands(new InstantCommand(() -> {
			drivetrain.resetOdometry(
					new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079), Rotation2d.fromDegrees(0)));
		}), new AutoDriveDistance(drivetrain, -0.35), new InstantCommand(
				() -> grabberSolenoidSubsystem.setGrabberPitchState(!grabberSolenoidSubsystem.getGrabberPitchState())),
				new SequentialCommandGroup(new CommandArmPitch(armSubsystem, 95, controls.overrideTeleopArm()),
						new CommandArmExtension(
								armSubsystem, ArmConstants.maxExtensionLengthEncCounts, controls.overrideTeleopArm())),
				new AutoDriveDistance(drivetrain, 0.35),
				new InstantCommand(() -> grabberSolenoidSubsystem.setGrabberIntakeState(
						!grabberSolenoidSubsystem.getGrabberIntakeState()), grabberSolenoidSubsystem),
				new WaitCommand(0.2),
				new SequentialCommandGroup(new ParallelCommandGroup(new CommandArmExtension(armSubsystem,
						ArmConstants.minExtensionLengthEncCounts, controls.overrideTeleopArm()),
						new InstantCommand(() -> {
							grabberSolenoidSubsystem.setGrabberIntakeState(false);
							grabberSolenoidSubsystem.setGrabberPitchState(false);
						}, grabberSolenoidSubsystem)),
						new CommandArm(armSubsystem, ArmConstants.minExtensionLengthEncCounts,
								ArmConstants.stowedDegrees, controls.overrideTeleopArm())),

				new AutoDriveDistance(drivetrain, -3.3),
				// Gets the robot out the comunity area (Over the charge station) by driving
				// backwards
				new InstantCommand(() -> {
					SectionManager.BODY.buffer.setSolidColor(Color.kRed);
					this.ledSubsystem.sendSectionData();
				}, ledSubsystem), new TurnAngle(drivetrain, () -> 180.0), // gets robot to pause outside of community
				new AutoDriveDistance(drivetrain, -1), new AutoBalance(drivetrain, true, ledSubsystem));
		// switch (this.startingPos) {

		// case LEFT :

		// addCommands(new InstantCommand(() -> {
		// drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42),
		// Units.inchesToMeters(42.079),
		// Rotation2d.fromDegrees(180)));
		// }),
		// new SequentialCommandGroup(new CommandArmPitch(armSubsystem, 95,
		// controls.overrideTeleopArm()),
		// new CommandArmExtension(armSubsystem, ArmConstants.maxExtensionLengthMillims,
		// controls.overrideTeleopArm())),
		// new SequentialCommandGroup(
		// new ParallelCommandGroup(new CommandArmExtension(armSubsystem,
		// ArmConstants.minExtensionLengthMillims,
		// controls.overrideTeleopArm()), new InstantCommand(() -> {
		// grabberSolenoidSubsystem.setGrabberIntakeState(false);
		// grabberSolenoidSubsystem.setGrabberPitchState(false);
		// }, grabberSolenoidSubsystem)),
		// new CommandArm(armSubsystem, ArmConstants.minExtensionLengthMillims,
		// ArmConstants.stowedDegrees,
		// controls.overrideTeleopArm())),
		// new AutoDriveDistance(drivetrain, 0.2),
		// new InstantCommand(() -> grabberSolenoidSubsystem
		// .setGrabberIntakeState(!grabberSolenoidSubsystem.getGrabberIntakeState())),
		// new AutoDriveDistance(drivetrain, -3.2),
		// // Gets the robot out the comunity area (Over the charge station) by driving
		// // backwards
		// new InstantCommand(() -> {
		// LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
		// LEDSubsystem.LEDManager.STRIP0.strip.sendData();
		// }, ledSubsystem), new WaitCommand(0.3), // gets robot to pause outside of
		// community
		// new AutoDriveDistance(drivetrain, 1), new AutoBalance(drivetrain, false,
		// ledSubsystem));

		// break;

		// case CENTER :

		// addCommands(new InstantCommand(() -> {
		// drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42),
		// Units.inchesToMeters(42.079),
		// Rotation2d.fromDegrees(180)));
		// }), new AutoDriveDistance(drivetrain, 2), new TurnDeltaAngle(drivetrain,
		// -90),
		// new AutoDriveDistance(drivetrain, 0.2), new TurnDeltaAngle(drivetrain, 90),
		// new AutoDriveDistance(drivetrain, 2), new AlignPeg(drivetrain),
		// new CommandArm(armSubsystem, 0.2, 90, controls.overrideTeleopArm()),
		// new GrabberInOutAuto(grabberMotorSubsystem, grabberSolenoidSubsystem, false,
		// 1),
		// new AutoDriveDistance(drivetrain, -2), new TurnDeltaAngle(drivetrain, -90),
		// new AutoDriveDistance(drivetrain, -0.2), new TurnDeltaAngle(drivetrain, 90),
		// new AutoDriveDistance(drivetrain, -2), new AutoDriveDistance(drivetrain,
		// -4.5),
		// // Gets the robot out the comunity area (Over the charge station) by driving
		// // backwards
		// new InstantCommand(() -> {
		// LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
		// LEDSubsystem.LEDManager.STRIP0.strip.sendData();
		// }, ledSubsystem), new WaitCommand(0.5), // gets robot to pause outside of
		// community
		// new AutoDriveDistance(drivetrain, 2), new AutoBalance(drivetrain, true,
		// ledSubsystem));

		// break;

		// case RIGHT :

		// addCommands(new InstantCommand(() -> {
		// drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42),
		// Units.inchesToMeters(42.079),
		// Rotation2d.fromDegrees(180)));
		// }), new AutoDriveDistance(drivetrain, 2), new TurnDeltaAngle(drivetrain,
		// -90),
		// new AutoDriveDistance(drivetrain, 0.2), new TurnDeltaAngle(drivetrain, 90),
		// new AutoDriveDistance(drivetrain, 2), new AlignPeg(drivetrain),
		// new CommandArm(armSubsystem, 0.2, 90, controls.overrideTeleopArm()),
		// new GrabberInOutAuto(grabberMotorSubsystem, grabberSolenoidSubsystem, false,
		// 1),
		// new AutoDriveDistance(drivetrain, -2), new TurnDeltaAngle(drivetrain, -90),
		// new AutoDriveDistance(drivetrain, -0.2), new TurnDeltaAngle(drivetrain, 90),
		// new AutoDriveDistance(drivetrain, -2), new AutoDriveDistance(drivetrain,
		// -4.5),
		// // Gets the robot out the comunity area (Over the charge station) by driving
		// // backwards
		// new InstantCommand(() -> {
		// LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
		// LEDSubsystem.LEDManager.STRIP0.strip.sendData();
		// }, ledSubsystem), new WaitCommand(0.5), // gets robot to pause outside of
		// community
		// new AutoDriveDistance(drivetrain, 2), new AutoBalance(drivetrain, true,
		// ledSubsystem));

		// break;
		// }

		// spotless:off

		// if (this.startingPos == StartingPosition.LEFT) {
		// 	addCommands(new InstantCommand(() -> {
		// 		drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079),
		// 				Rotation2d.fromDegrees(180)));
		// 	}), new AutoDriveDistance(drivetrain, 0.5), new TurnDeltaAngle(drivetrain, -90),
		// 			new AutoDriveDistance(drivetrain, 0.2), new TurnDeltaAngle(drivetrain, 90),
		// 			new AutoDriveDistance(drivetrain, 0.5), new AlignPeg(drivetrain),
		// 			new CommandArm(armSubsystem, 0.2, 90),
		// 			new GrabberInOutAuto(grabberMotorSubsystem, grabberSolenoidSubsystem, false, 1),
		// 			new AutoDriveDistance(drivetrain, -0.5), new TurnDeltaAngle(drivetrain, -90),
		// 			new AutoDriveDistance(drivetrain, -0.2), new TurnDeltaAngle(drivetrain, 90),
		// 			new AutoDriveDistance(drivetrain, -0.5), new AutoDriveDistance(drivetrain, -4.5),
		// 			// Gets the robot out the comunity area (Over the charge station) by driving
		// 			// backwards
		// 			new InstantCommand(() -> {
		// 				LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
		// 			}), new WaitCommand(0.5), // gets robot to pause outside of community
		// 			new AutoDriveDistance(drivetrain, 2), new AutoBalance(drivetrain, true, ledSubsystem));

		// } else if (this.startingPos == StartingPosition.CENTER) {
		// 	addCommands(new InstantCommand(() -> {
		// 		drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079),
		// 				Rotation2d.fromDegrees(180)));
		// 	}), new AutoDriveDistance(drivetrain, 2), new TurnDeltaAngle(drivetrain, -90),
		// 			new AutoDriveDistance(drivetrain, 0.2), new TurnDeltaAngle(drivetrain, 90),
		// 			new AutoDriveDistance(drivetrain, 2), new AlignPeg(drivetrain),
		// 			new CommandArm(armSubsystem, 0.2, 90),
		// 			new GrabberInOutAuto(grabberMotorSubsystem, grabberSolenoidSubsystem, false, 1),
		// 			new AutoDriveDistance(drivetrain, -2), new TurnDeltaAngle(drivetrain, -90),
		// 			new AutoDriveDistance(drivetrain, -0.2), new TurnDeltaAngle(drivetrain, 90),
		// 			new AutoDriveDistance(drivetrain, -2), new AutoDriveDistance(drivetrain, -4.5),
		// 			// Gets the robot out the comunity area (Over the charge station) by driving
		// 			// backwards
		// 			new InstantCommand(() -> {
		// 				LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
		// 			}), new WaitCommand(0.5), // gets robot to pause outside of community
		// 			new AutoDriveDistance(drivetrain, 2), new AutoBalance(drivetrain, true, ledSubsystem));
		// } else {
		// 	addCommands(new InstantCommand(() -> {
		// 		drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters(54.42), Units.inchesToMeters(42.079),
		// 				Rotation2d.fromDegrees(180)));
		// 	}), new AutoDriveDistance(drivetrain, 2), new TurnDeltaAngle(drivetrain, -90),
		// 			new AutoDriveDistance(drivetrain, 0.2), new TurnDeltaAngle(drivetrain, 90),
		// 			new AutoDriveDistance(drivetrain, 2), new AlignPeg(drivetrain),
		// 			new CommandArm(armSubsystem, 0.2, 90),
		// 			new GrabberInOutAuto(grabberMotorSubsystem, grabberSolenoidSubsystem, false, 1),
		// 			new AutoDriveDistance(drivetrain, -2), new TurnDeltaAngle(drivetrain, -90),
		// 			new AutoDriveDistance(drivetrain, -0.2), new TurnDeltaAngle(drivetrain, 90),
		// 			new AutoDriveDistance(drivetrain, -2), new AutoDriveDistance(drivetrain, -4.5),
		// 			// Gets the robot out the comunity area (Over the charge station) by driving
		// 			// backwards
		// 			new InstantCommand(() -> {
		// 				LEDSubsystem.LEDManager.STRIP0.strip.setSolidColor(Color.kRed);
		// 			}), new WaitCommand(0.5), // gets robot to pause outside of community
		// 			new AutoDriveDistance(drivetrain, 2), new AutoBalance(drivetrain, true, ledSubsystem));
		// }

		// spotless:on
	}

}
