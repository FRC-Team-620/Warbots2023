package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmCommand extends CommandBase{

  ArmSubsystem armSubsystem;
  private CommandXboxController controller;
  
  //Constructor 
  public ArmCommand(ArmSubsystem armSubsystem){
    this.armSubsystem = armSubsystem;
  }

  // execute
  @Override
  public void execute() {
    //armSubsystem.setSolenoid(!this.armSubsystem.getSolenoid());
    //enables the controller inputs for arm subsystem 
    double controlLength = Math.pow(controller.getRightX(), 2);
    double controlHeight = Math.pow(controller.getRightY(), 2);



    //enables the motors to control their respective jobs
    armSubsystem.setHorizontalArmMotor(controlLength);
    armSubsystem.setVerticalArmMotor(controlHeight);

  }

  // isFinished
  @Override
  public boolean isFinished() {
    return false;
  }
}


