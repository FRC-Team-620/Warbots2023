package org.jmhsrobotics.frc2023.util;

public class ArmConversion {

    public ArmConversion()
    {

    }

    public double encoderToAngle(double encoder)
    {
        return -114.440596296 * (encoder - 2.49) - 63;
    }
    
    public double angleToEncoder(double angle)
    {
        return (angle + 63)/-114.440596296 + 2.49;
    }
}
