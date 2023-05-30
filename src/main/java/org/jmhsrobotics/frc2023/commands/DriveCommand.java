// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023.commands;

import org.jmhsrobotics.frc2023.oi.ControlBoard;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An DriveCommand command that uses an Drivetrain subsystem. */
public class DriveCommand extends CommandBase {
	private Drivetrain drivetrain;
	private ControlBoard control;
	// private CommandXboxController controller;
	private boolean rightStick = false;

	/**
	 * Creates a new DriveCommand.
	 *
	 * @param drivetrain
	 *            The subsystem used by this command.
	 */
	public DriveCommand(Drivetrain drivetrain, ControlBoard board) {
		this.drivetrain = drivetrain;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);

		this.control = board;

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		SmartDashboard.putNumber("Drivetrain/drive speed", 0.15);
		SmartDashboard.putNumber("Drivetrain/turning speed", 0.3);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Create 3 doubles for left trigger input, right trigger input, and the left
		// stick of the XboxController's x axis
		// We squared all the inputs last season for finer control at lower speeds and
		// the drivers seemed to like it so we probably should keep it as it's what
		// everyone is used to
		// Squared input: allows you to have more control when moving slowly but be able
		// to move fast quickly
		// This was the input scheme we went with last season in 2022

		// double leftTriggerInput = Math.pow(controller.getLeftTriggerAxis(), 2);//used
		// for backward movement
		// double rightTriggerInput = Math.pow(controller.getRightTriggerAxis(),
		// 2);//used for forward movement
		double rotationInput = control.driveTurn();

		// After that you should multiply the rotation input number by -1 if the input
		// was negative and by nothing if positive, you could use signum for this if you
		// want to make it short.
		// You have to do this to make sure the rotation is in the proper range as
		// squaring the input gets rid of the negative on the number and makes it a
		// positive

		// Create a boolean called quickTurn that equals true (quickTurn is used to give
		// the drivers the option to use CurvertureDrive or not)
		// Make an if statement that checks if the controller has the A button held and
		// sets quickTurn to false
		boolean quickTurn = control.isQuickTurn(); // Used for tank steering if true

		double speed = 0;
		// Make an if statement that triggers if the left trigger is pressed down more
		// then the right trigger and sets the speed to be equal to -left trigger and
		// else the speed is set to right trigger

		// The if statement allows for the left and right inputs to be pressed down at
		// the same time but the one pressed down more
		// controls the bot
		speed = control.driveForward();

		// speed *= 0.5;
		// rotationInput *= 0.5;

		// if (control.isDriveSlow()) {
		// speed /= 2;
		// rotationInput /= 2;
		// }

		if (control.isDriveFast()) {
			// System.out.println("high");
			speed *= 1;
			rotationInput *= 0.7;
		} else {
			// System.out.println("slow");
			// System.out.println(SmartDashboard.getNumber("Drivetrain/drive speed", 0.15));
			// System.out.println(SmartDashboard.getNumber("Drivetrain/drive rotation
			// speed", 0.3));
			speed *= 0.2;
			rotationInput *= 0.25;
		}
		// Pass the speed, rotation input, and the quickTurn in that order into
		// setCurvatureDrive
		// This will allow for Drivetrain's DifferentalDrive to assign the motors to the
		// correct values to make that movement
		drivetrain.setCurvatureDrive(speed, rotationInput, quickTurn);
		SmartDashboard.putNumber("right motor encoder", drivetrain.getRightEncoderCount());
		SmartDashboard.putNumber("left motor encoder", drivetrain.getLeftEncoderCount());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
