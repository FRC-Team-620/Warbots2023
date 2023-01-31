package frc.robot.util;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.IArmPorts;

public class ArmPorts implements IArmPorts {

    public final int armVerticalCANId = 5;
    public final int armHorizontalCANId = 6;

    //public final boolean rightFrontMotorInversion = false;
    //public final boolean leftFrontMotorInversion = true;

    //public final double wheelDiameterInInches = 6;

    
    @Override
    public int getArmVerticalCANId(){
       return armVerticalCANId;
    }
}