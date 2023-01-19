package frc.robot;

import java.lang.Math;
import frc.robot.Constants.RobotMathConstants;

public class RobotMath {

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
