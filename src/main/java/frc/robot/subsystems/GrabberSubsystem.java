package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.armPorts;

public class GrabberSubsystem extends SubsystemBase {

    //private  grabber
    
    public void setGrabberState(boolean closed){
        armHorizontal.set(amount);
    }
}
