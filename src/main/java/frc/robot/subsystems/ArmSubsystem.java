package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
Compressor phCompressor = new Compressor(42, PneumaticsModuleType.REVPH);
private Solenoid solenoid = new Solenoid(42, PneumaticsModuleType.REVPH, 1);

//Constructor 
public ArmSubsystem() {
    phCompressor.enableDigital();
    
}


//
public void setSolenoid(boolean state){
    solenoid.set(state);
}

public boolean getSolenoid(){
    return solenoid.get();
}
  
}

