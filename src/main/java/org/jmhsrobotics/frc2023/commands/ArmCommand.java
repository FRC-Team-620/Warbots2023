package org.jmhsrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

	ArmSubsystem armSubsystem;
	private XboxController controller;

	// Constructor
	public ArmCommand(ArmSubsystem armSubsystem, XboxController controller) {
		this.armSubsystem = armSubsystem;
		addRequirements(armSubsystem);
		this.controller = controller;
	}

	// execute
	@Override
	public void execute() {
		// armSubsystem.setSolenoid(!this.armSubsystem.getSolenoid());
		// enables the controller inputs for arm subsystem
		double controlLength = MathUtil.interpolate(0, Units.inchesToMeters(45), (controller.getRightY() + 1) / 2.0);
		double controlPitch = MathUtil.interpolate(-180, 0, (controller.getRightX() + 1) / 2.0);

		// enables the motors to control their respective jobs
		armSubsystem.setExtensionArmMotor(controlLength);
		armSubsystem.setPitchArmMotor(controlPitch);
	}

	// isFinished
	@Override
	public boolean isFinished() {
		return false;
	}
}
