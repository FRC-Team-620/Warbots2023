package frc.robot;

import java.lang.Math;
import frc.robot.Constants.RobotMathConstants;

public class RobotMath {

    /** 
     * For systems where angles are given in the range [-180, 180]:
     * Returns the angle of some target angle relative to some pivot angle
     * That is, it returns the angle of the target angle as though the
     * pivot angle were at 0 degrees.
     * 
     * @param pivotAngle The angle that the target angle is relative to
     * @param targetAngle The angle whose relative angle to the pivot angle will be returned
     * @return  The relative angle to the target angle (i.e. if the pivot angle were at 0)
    */
    public static double relativeAngle(double pivotAngle, double targetAngle) {
        double diff = targetAngle - pivotAngle;
        if(Math.abs(diff) <= 180.0) 
            return diff;
        return diff - Math.signum(diff) * 360.0;
    }

    public static boolean approximatelyZero(double value) {
        return value > -RobotMathConstants.comparisonThreshhold && 
            value < RobotMathConstants.comparisonThreshhold;
    }
}
