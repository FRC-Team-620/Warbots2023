// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An DriveCommand command that uses an Drivetrain subsystem. */
public class DriveCommand extends CommandBase {
  private Drivetrain drivetrain;
  private CommandXboxController controller;
  /**
   * Creates a new DriveCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
  public DriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.controller = controller;
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Create 3 doubles for left trigger input, right trigger input, and the left stick of the XboxController's x axis
    // We squared all the inputs last season for finer control at lower speeds and the drivers seemed to like it so we probably should keep it as it's what everyone is used to
    //Squared input: allows you to have more control when moving slowly but be able to move fast quickly
    //This was the input scheme we went with last season in 2022
    double leftTriggerInput = Math.pow(controller.getLeftTriggerAxis(), 2);//used for backward movement
    double rightTriggerInput = Math.pow(controller.getRightTriggerAxis(), 2);//used for forward movement
    double rotationInput = Math.pow(controller.getLeftX(), 2);
    rotationInput *= Math.signum(controller.getLeftX());//This is either -1 if the input is a negative or 1 if the input is a positive 

    // Make an if statement that checks if the controller has the A button held and sets quickTurn to false
    boolean quickTurn = true;//Used for tank steering if true
    if (controller.a().getAsBoolean()) {//if the A button is held then tank steering is enabled
      quickTurn = false;
    }
    double speed = 0;
    // Make an if statement that triggers if the left trigger is pressed down more then the right trigger and sets the speed to be equal to -left trigger and else the speed is set to right trigger
    

    //The if statement allows for the left and right inputs to be pressed down at the same time but the one pressed down more
    //controls the bot
    //rightfront, rightback
    speed = rightTriggerInput > leftTriggerInput ? rightTriggerInput : -leftTriggerInput;
    

    // Pass the speed, rotation input, and the quickTurn in that order into setCurvatureDrive
    // This will allow for Drivetrain's DifferentalDrive to assign the motors to the correct values to make that movement

    drivetrain.setCurvatureDrive(speed, rotationInput, quickTurn);
    //drivetrain.setRightMotors(1);
    //drivetrain.setLeftMotors(1);

    SmartDashboard.putNumber("left encoder", drivetrain.getLeftEncoderCount());
    SmartDashboard.putNumber("right encoder", drivetrain.getRightEncoderCount());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
