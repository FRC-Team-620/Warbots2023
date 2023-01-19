package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase{
    ArmSubsystem armSubsystem = new ArmSubsystem();
    //Constructor 
    public ArmCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
    }

    // execute
  @Override
  public void execute() {
    if(this.armSubsystem.getSolenoid()){
        armSubsystem.setSolenoid(false);
    } else {
        armSubsystem.setSolenoid(true);
    }
  }

  // isFinished
  @Override
  public boolean isFinished() {
    
    return true;
  }
}


