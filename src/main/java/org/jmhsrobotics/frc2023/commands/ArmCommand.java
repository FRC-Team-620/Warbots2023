package org.jmhsrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

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


