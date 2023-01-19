package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
private Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

//Constructor 
public ArmSubsystem(){

}


//
public void setSolenoid(boolean state){
    solenoid.set(state);
}

public boolean getSolenoid(){
    return solenoid.get();
}
  
}

