package org.jmhsrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmCommand extends CommandBase {

	ArmSubsystem armSubsystem;
	private CommandXboxController operatorController;

	// Constructor
	public ArmCommand(ArmSubsystem armSubsystem, CommandXboxController controller) {
		this.armSubsystem = armSubsystem;
		this.operatorController = controller;
		addRequirements(armSubsystem);
	}

	// execute
	@Override
	public void execute() {
		// armSubsystem.setSolenoid(!this.armSubsystem.getSolenoid());
		// enables the controller inputs for arm subsystem
		double controlLength = Math.pow(operatorController.getRightX(), 2);
		double controlHeight = Math.pow(operatorController.getRightY(), 2);

		// enables the motors to control their respective jobs
		armSubsystem.setHorizontalArmMotor(controlLength);
		armSubsystem.setVerticalArmMotor(controlHeight);

	}

	// isFinished
	@Override
	public boolean isFinished() {
		return false;
	}
}
