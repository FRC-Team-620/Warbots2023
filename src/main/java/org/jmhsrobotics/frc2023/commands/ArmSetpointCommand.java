package org.jmhsrobotics.frc2023.commands;

import org.jmhsrobotics.frc2023.RobotContainer;
import org.jmhsrobotics.frc2023.Constants.ArmConstants;
import org.jmhsrobotics.frc2023.Constants.GripperType;
import org.jmhsrobotics.frc2023.Constants.Setpoints;
import org.jmhsrobotics.frc2023.Constants.WristConstants;
import org.jmhsrobotics.frc2023.commands.arm.CommandAThenEW;
import org.jmhsrobotics.frc2023.commands.arm.CommandAWThenE;
import org.jmhsrobotics.frc2023.commands.arm.CommandArmPitchThenExtension;
import org.jmhsrobotics.frc2023.commands.arm.CommandEThenAW;
import org.jmhsrobotics.frc2023.commands.gripper.CommandIntakeSolenoid;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmSetpointCommand extends SequentialCommandGroup {
	// Setpoints setpoint;
	public ArmSetpointCommand(Setpoints setpoint, GripperType gripperType, RobotContainer robotContainer) {
		switch (gripperType) {
			case SOLENOID :
				switch (setpoint) {
					case STOWED :
						addCommands(new ConditionalCommand(new ParallelCommandGroup(
								new CommandIntakeSolenoid(robotContainer.getIntakeSubsystem(), false),
								new CommandEThenAW(robotContainer.getArmSubsystem(), robotContainer.getWristSubsystem(),
										0.0, ArmConstants.stowedDegrees, WristConstants.stowedPositionAbsolute,
										robotContainer.getControlBoard().override())),
								new ParallelCommandGroup(
										new CommandIntakeSolenoid(robotContainer.getIntakeSubsystem(), true),
										new CommandEThenAW(robotContainer.getArmSubsystem(),
												robotContainer.getWristSubsystem(), 0.0, ArmConstants.stowedDegrees,
												WristConstants.stowedPositionAbsolute,
												robotContainer.getControlBoard().override())),
								robotContainer.getArmSubsystem()::isCone));
						break;
					case FLOOR :
						addCommands(new ConditionalCommand(new ParallelCommandGroup(
								new CommandIntakeSolenoid(robotContainer.getIntakeSubsystem(), false),
								new CommandAWThenE(robotContainer.getArmSubsystem(), robotContainer.getWristSubsystem(),
										0.8, 66, 90, robotContainer.getControlBoard().override())),
								new ParallelCommandGroup(
										new CommandIntakeSolenoid(robotContainer.getIntakeSubsystem(), true),
										new CommandAWThenE(robotContainer.getArmSubsystem(),
												robotContainer.getWristSubsystem(), 0.8, 45, 90,
												robotContainer.getControlBoard().override())),
								robotContainer.getArmSubsystem()::isCone));
						break;
					case MID :
						addCommands(new ConditionalCommand(
								new CommandAThenEW(robotContainer.getArmSubsystem(), robotContainer.getWristSubsystem(),
										0.8, 95, 90, robotContainer.getControlBoard().override()),
								new CommandAThenEW(robotContainer.getArmSubsystem(), robotContainer.getWristSubsystem(),
										0.8, 80, 90, robotContainer.getControlBoard().override()),
								robotContainer.getArmSubsystem()::isCone));
						break;
					case HIGH :
						addCommands(new ConditionalCommand(
								new CommandArmPitchThenExtension(robotContainer.getArmSubsystem(), 1.0, 247,
										robotContainer.getControlBoard().override()),
								new CommandArmPitchThenExtension(robotContainer.getArmSubsystem(), 1.0, 247,
										robotContainer.getControlBoard().override()),
								robotContainer.getArmSubsystem()::isCone));
						break;

					case PICKUP :
						addCommands(new ConditionalCommand(new ParallelCommandGroup(
								new CommandEThenAW(robotContainer.getArmSubsystem(), robotContainer.getWristSubsystem(),
										0.0, 67, 120, robotContainer.getControlBoard().override()),
								new CommandIntakeSolenoid(robotContainer.getIntakeSubsystem(), false)),
								new ParallelCommandGroup(
										new CommandEThenAW(robotContainer.getArmSubsystem(),
												robotContainer.getWristSubsystem(), 0.0, 67, 120,
												robotContainer.getControlBoard().override()),
										new CommandIntakeSolenoid(robotContainer.getIntakeSubsystem(), true)),
								robotContainer.getArmSubsystem()::isCone));
						break;
					default :
						break;
				}
				break;

			case MOTOR :
				switch (setpoint) {
					case STOWED :
						addCommands(new ConditionalCommand(
								new ParallelCommandGroup(new CommandEThenAW(robotContainer.getArmSubsystem(),
										robotContainer.getWristSubsystem(), 0.0, ArmConstants.stowedDegrees,
										WristConstants.stowedPositionAbsolute,
										robotContainer.getControlBoard().override())),
								new ParallelCommandGroup(new CommandEThenAW(robotContainer.getArmSubsystem(),
										robotContainer.getWristSubsystem(), 0.0, ArmConstants.stowedDegrees,
										WristConstants.stowedPositionAbsolute,
										robotContainer.getControlBoard().override())),
								robotContainer.getArmSubsystem()::isCone));
						break;
					case FLOOR :
						addCommands(new ConditionalCommand(new ParallelCommandGroup(
								new CommandIntakeSolenoid(robotContainer.getIntakeSubsystem(), false),
								new CommandAWThenE(robotContainer.getArmSubsystem(), robotContainer.getWristSubsystem(),
										0.0, 43, 60, robotContainer.getControlBoard().override())),
								new ParallelCommandGroup(new CommandAWThenE(robotContainer.getArmSubsystem(),
										robotContainer.getWristSubsystem(), 0.8, 35, 65,
										robotContainer.getControlBoard().override())),
								robotContainer.getArmSubsystem()::isCone));
						break;
					case MID :
						addCommands(new ConditionalCommand(
								new CommandAThenEW(robotContainer.getArmSubsystem(), robotContainer.getWristSubsystem(),
										0.2, 95, 90, robotContainer.getControlBoard().override()),
								new CommandAThenEW(robotContainer.getArmSubsystem(), robotContainer.getWristSubsystem(), // If
																															// it
																															// gets
																															// stuck
																															// replace
																															// second
																															// command
																															// with
																															// commented
																															// out
																															// code
										0.75, 84, 90, robotContainer.getControlBoard().override()),
								// new ConditionalCommand(
								// new CommandAThenEW(robotContainer.getArmSubsystem(),
								// robotContainer.getWristSubsystem(),
								// 0.75, 84, 90, robotContainer.getControlBoard().override()),
								// new SequentialCommandGroup(
								// new CommandAWThenE(robotContainer.getArmSubsystem(),
								// robotContainer.getWristSubsystem(), 1.0, 48, 58,
								// robotContainer.getControlBoard().override()),
								// new CommandAThenEW(robotContainer.getArmSubsystem(),
								// robotContainer.getWristSubsystem(),
								// 0.75, 84, 90, robotContainer.getControlBoard().override()),

								// , robotContainer.getArmSubsystem()::isHigherThen),

								robotContainer.getArmSubsystem()::isCone));
						break;
					case HIGH :
						addCommands(// Cones
								new ConditionalCommand(new SequentialCommandGroup(
										// new InstantCommand(() ->
										// robotContainer.getArmSubsystem().setAnglePIDConstraints(new
										// TrapezoidProfile.Constraints(110, 150)),
										// robotContainer.getArmSubsystem()),
										new CommandAWThenE(robotContainer.getArmSubsystem(),
												robotContainer.getWristSubsystem(), 0.0,
												robotContainer.getArmSubsystem()::getArmPitch,
												robotContainer::getAbsoluteStow,
												robotContainer.getControlBoard().override()),

										new CommandAThenEW(robotContainer.getArmSubsystem(),
												robotContainer.getWristSubsystem(), 1.0, 240, 290, // If arm
																									// angle is
																									// too
																									// shallow
																									// for arm
																									// to come
																									// out make
																									// the arm
																									// angle
																									// lower
												robotContainer.getControlBoard().override()),

										new CommandAThenEW(robotContainer.getArmSubsystem(),
												robotContainer.getWristSubsystem(), 1.0, 221, 271,
												robotContainer.getControlBoard().override())),
										// Cubes
										new ConditionalCommand(
												new CommandAThenEW(robotContainer.getArmSubsystem(),
														robotContainer.getWristSubsystem(), 1.0, 103, 113,
														robotContainer.getControlBoard().override()),
												new SequentialCommandGroup(
														// new CommandAWThenE(robotContainer.getArmSubsystem(),
														// robotContainer.getWristSubsystem(), 1.0, 48, 58,
														// robotContainer.getControlBoard().override()),
														new CommandAWThenE(robotContainer.getArmSubsystem(),
																robotContainer.getWristSubsystem(), 1.0, 103, 113,
																robotContainer.getControlBoard().override()))

												, robotContainer.getArmSubsystem()::isHigherThan),
										// new CommandArmPitchThenExtension(robotContainer.armSubsystem, 1.0, 247,
										// robotContainer.controlBoard.override()),33
										// new CommandArmPitchThenExtension(robotContainer.armSubsystem, 1.0, 247,
										// robotContainer.controlBoard.override()),
										robotContainer.getArmSubsystem()::isCone));
						break;

					case PICKUP :
						addCommands(new ConditionalCommand(
								new ParallelCommandGroup(new CommandEThenAW(robotContainer.getArmSubsystem(),
										robotContainer.getWristSubsystem(), 0.0, 48, 158,
										robotContainer.getControlBoard().override())),
								new ParallelCommandGroup(new CommandEThenAW(robotContainer.getArmSubsystem(),
										robotContainer.getWristSubsystem(), 0.0, 67, 140,
										robotContainer.getControlBoard().override())),
								robotContainer.getArmSubsystem()::isCone));
						break;
					default :
						break;
				}
				break;
		}

		// addCommands();
	}
}
