package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase{

  ArmSubsystem armSubsystem;

  //Constructor 
  public ArmCommand(ArmSubsystem armSubsystem){
    this.armSubsystem = armSubsystem;
  }

  // execute
  @Override
  public void execute() {
    armSubsystem.setSolenoid(!this.armSubsystem.getSolenoid());
  }

  // isFinished
  @Override
  public boolean isFinished() {
    return true;
  }
}


