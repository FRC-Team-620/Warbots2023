package org.jmhsrobotics.frc2023.commands;

import org.jmhsrobotics.frc2023.RobotContainer;
import org.jmhsrobotics.frc2023.Constants.ArmConstants;
import org.jmhsrobotics.frc2023.Constants.WristConstants;
import org.jmhsrobotics.frc2023.commands.arm.CommandAThenEW;
import org.jmhsrobotics.frc2023.commands.arm.CommandAWThenE;
import org.jmhsrobotics.frc2023.commands.arm.CommandArmPitchThenExtension;
import org.jmhsrobotics.frc2023.commands.arm.CommandEThenAW;
import org.jmhsrobotics.frc2023.commands.arm.CommandEWThenA;
import org.jmhsrobotics.frc2023.commands.grabber.CommandIntakeSolenoid;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmSetpointCommand extends SequentialCommandGroup {
    public enum Setpoints {
        STOWED, FLOOR, MID, HIGH, PICKUP
    }
    public enum GripperType {
        SOLENOID, MOTOR 
    }
    //Setpoints setpoint;
    public ArmSetpointCommand(Setpoints setpoint, GripperType gripperType, RobotContainer robotContainer) {
        switch (gripperType) {
            case SOLENOID:
            switch(setpoint) {
                case STOWED:
                    addCommands(new ConditionalCommand(
                        new ParallelCommandGroup(new CommandIntakeSolenoid(robotContainer.intakeSubsystem, false),
                                new CommandEWThenA(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.0, ArmConstants.stowedDegrees,
                                        WristConstants.stowedPositionAbsolute, robotContainer.controlBoard.override())),
                        new ParallelCommandGroup(new CommandIntakeSolenoid(robotContainer.intakeSubsystem, true),
                                new CommandEWThenA(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.0, ArmConstants.stowedDegrees,
                                        WristConstants.stowedPositionAbsolute, robotContainer.controlBoard.override())),
                                        robotContainer.armSubsystem::isCone));
                    break;
                case FLOOR:
                    addCommands(new ConditionalCommand(
						new ParallelCommandGroup(new CommandIntakeSolenoid(robotContainer.intakeSubsystem, false),
								new CommandAWThenE(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.8, 35, 90, robotContainer.controlBoard.override())),
						new ParallelCommandGroup(new CommandIntakeSolenoid(robotContainer.intakeSubsystem, true),
								new CommandAWThenE(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.8, 35, 90, robotContainer.controlBoard.override())),
                                robotContainer.armSubsystem::isCone));
                    break;
                case MID:
                    addCommands(new ConditionalCommand(
						new CommandAThenEW(robotContainer.armSubsystem, robotContainer.wristSubsystem, 1.0, 95, 90, robotContainer.controlBoard.override()),
						new CommandAThenEW(robotContainer.armSubsystem, robotContainer.wristSubsystem, 1.0, 80, 90, robotContainer.controlBoard.override()),
						robotContainer.armSubsystem::isCone));
                    break;
                case HIGH:
                    addCommands(new ConditionalCommand(
						new CommandArmPitchThenExtension(robotContainer.armSubsystem, 1.0, 247, robotContainer.controlBoard.override()),
						new CommandArmPitchThenExtension(robotContainer.armSubsystem, 1.0, 247, robotContainer.controlBoard.override()),
						robotContainer.armSubsystem::isCone));
                    break;
                
                case PICKUP:
                    addCommands(new ConditionalCommand(
                        new ParallelCommandGroup(
                                new CommandEThenAW(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.0, 67, 120,
                                robotContainer.controlBoard.override()),
                                new CommandIntakeSolenoid(robotContainer.intakeSubsystem, false)),
                        new ParallelCommandGroup(
                                new CommandEThenAW(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.0, 67, 120,
                                robotContainer.controlBoard.override()),
                                new CommandIntakeSolenoid(robotContainer.intakeSubsystem, true)),
                                robotContainer.armSubsystem::isCone));
                    break;
                default:
                    break;
            }
                break;
            case MOTOR:
            switch(setpoint) {
                case STOWED:
                    addCommands(new ConditionalCommand(
                        new ParallelCommandGroup(
                                new CommandEWThenA(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.0, ArmConstants.stowedDegrees,
                                        WristConstants.stowedPositionAbsolute, robotContainer.controlBoard.override())),
                        new ParallelCommandGroup(
                                new CommandEWThenA(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.0, ArmConstants.stowedDegrees,
                                        WristConstants.stowedPositionAbsolute, robotContainer.controlBoard.override())),
                                        robotContainer.armSubsystem::isCone));
                    break;
                case FLOOR:
                    addCommands(new ConditionalCommand(
						new ParallelCommandGroup(
								new CommandAWThenE(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.8, 35, 90, robotContainer.controlBoard.override())),
						new ParallelCommandGroup(
								new CommandAWThenE(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.8, 35, 90, robotContainer.controlBoard.override())),
                                robotContainer.armSubsystem::isCone));
                    break;
                case MID:
                    addCommands(new ConditionalCommand(
						new CommandAThenEW(robotContainer.armSubsystem, robotContainer.wristSubsystem, 1.0, 95, 90, robotContainer.controlBoard.override()),
						new CommandAThenEW(robotContainer.armSubsystem, robotContainer.wristSubsystem, 1.0, 80, 90, robotContainer.controlBoard.override()),
						robotContainer.armSubsystem::isCone));
                    break;
                case HIGH:
                    addCommands(new ConditionalCommand(
						new CommandArmPitchThenExtension(robotContainer.armSubsystem, 1.0, 247, robotContainer.controlBoard.override()),
						new CommandArmPitchThenExtension(robotContainer.armSubsystem, 1.0, 247, robotContainer.controlBoard.override()),
						robotContainer.armSubsystem::isCone));
                    break;
                
                case PICKUP:
                    addCommands(new ConditionalCommand(
                        new ParallelCommandGroup(
                                new CommandEThenAW(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.0, 67, 120,
                                robotContainer.controlBoard.override())),
                        new ParallelCommandGroup(
                                new CommandEThenAW(robotContainer.armSubsystem, robotContainer.wristSubsystem, 0.0, 67, 120,
                                robotContainer.controlBoard.override())),
                                robotContainer.armSubsystem::isCone));
                    break;
                default:
                    break;
            }
                break;
        }
        
        //addCommands();
    }
}
