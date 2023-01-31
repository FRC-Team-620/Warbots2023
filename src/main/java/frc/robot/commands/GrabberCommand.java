package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GrabberCommand extends CommandBase{

  GrabberSubsystem grabberSubsystem;
  private CommandXboxController controller;
  
  //Constructor 
  public GrabberCommand(ArmSubsystem armSubsystem){
    this.armSubsystem = armSubsystem;
  }

  // execute
  @Override
  public void execute() {
    grabberSubsystem.setSolenoid(!this.armSubsystem.getSolenoid());
    //enables the controller inputs for arm subsystem 
    double controlLength = Math.pow(controller.getRightX(), 2);
    double controlHeight = Math.pow(controller.getRightY(), 2);
    
    boolean grabberClosed=controller.b().getAsBoolean();
    
    

    //enables the motors to control their respective jobs

  }

  // isFinished
  @Override
  public boolean isFinished() {
    return false;
  }
}


