package frc.robot;

import java.lang.Math;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.RobotMathConstants;

public class RobotMath {

    /** 
     * For systems where angles are given in the domain [-180, 180]:
     * Returns the angle of some target angle relative to some pivot angle
     * That is, it returns the angle of the target angle as though the
     * pivot angle were at 0 degrees. This accounts for the restriction that
     * angles must be in this specific domain.
     * 
     * @param pivotAngle The angle that the target angle is relative to
     * @param targetAngle The angle whose relative angle to the pivot angle will be returned
     * @return  The relative angle to the target angle (i.e. if the pivot angle were at 0) in
     * the domain [-180, 180]
     */
    public static double relativeAngle(double pivotAngle, double targetAngle) {
        double diff = targetAngle - pivotAngle;
        if(Math.abs(diff) <= 180.0) 
            return diff;
        return diff - Math.signum(diff) * 360.0;
    }

    /**
     * For systems where angles are given in the domain [-180, 180]:
     * Returns the resultant angle from adding some delta angle to a pivot angle, accounting 
     * for the restriction that angles must be in this specific domain.
     * 
     * @param pivotAngle The angle that the deltaAngle is being added to (shifted)
     * @param deltaAngle The angle being added to pivot angle (the shift)
     * @return The resultant angle of the shift in the domain [-180, 180]
     */
    public static double shiftAngle(double pivotAngle, double deltaAngle) {
        return RobotMath.relativeAngle(-pivotAngle, deltaAngle);
    }

    /** 
     * Checking whether a number appoximates zero based on the
     * threshholds set in Constants.RobotMathConstants
     * 
     * @param value The value to be checked
     * @return  True if the value approximates zero, false otherwise
     */
    public static boolean approximatelyZero(double value) {
        return RobotMath.approximatelyZero(value, RobotMathConstants.comparisonThreshhold);
    }

    /** 
     * Checking whether a number appoximates zero based on the
     * threshhold given
     * 
     * @param value The value to be checked
     * @param threshhold Determines how close the value needs to be to zero to pass
     * @return  True if the value approximates zero, false otherwise
     */
    public static boolean approximatelyZero(double value, double threshhold) {
        return value > -threshhold && value < threshhold;
    }

    public static double constrain180(double angle) {
        return MathUtil.inputModulus(angle, -180, 180);
    }

    public static class DiminishingAverageHandler {
    
        private java.lang.Double average = null; // for nullability
        private final double previousWeight;
    
        /**
         * Handles running averages where past data should be expelled over time.
         * As new data comes in, the previous data is reduced in weight by some factor, 
         * and over time, this makes old data approach irrelevancy.
         * 
         * @param previousWeight The factor that previous data is multiplied by. This 
         * determines how important past data is. The larger, the more important. The closer
         * to zero, the less important.
         */
        public DiminishingAverageHandler(double previousWeight) {
            this.previousWeight = previousWeight;
        }
    
        /**
         * Getter for the current average
         * 
         * @return The current average
         */
        public double get() {
            return this.average;
        }
    
        /**
         * Give the DiminishingAverageHandler new data and return the new average.
         * If this is the first datum, then this will be returned as is.
         * 
         * @param data The new data
         * @return The new average
         */
        public double feed(double data) {
            if(average == null) {
                this.average = data;
                return this.average;
            }
            this.average = (this.average * this.previousWeight + data) / (1.0 + this.previousWeight);
            return average;
        }
    
        /**
         * Resets the DimishingAverageHandler, deleting past data
         */
        public void reset() {
            this.average = null;
        }
    
    }
}