package org.jmhsrobotics.frc2023;

import edu.wpi.first.math.MathUtil;

import java.util.function.ToDoubleFunction;
import java.util.function.ToIntFunction;

import org.jmhsrobotics.frc2023.Constants.RobotMathConstants;

public class RobotMath {

	/**
	 * For systems where angles are given in the domain [-180, 180]: Returns the
	 * angle of some target angle relative to some pivot angle That is, it returns
	 * the angle of the target angle as though the pivot angle were at 0 degrees.
	 * This accounts for the restriction that angles must be in this specific
	 * domain.
	 *
	 * @param pivotAngle
	 *            The angle that the target angle is relative to
	 * @param targetAngle
	 *            The angle whose relative angle to the pivot angle will be returned
	 * @return The relative angle to the target angle (i.e. if the pivot angle were
	 *         at 0) in the domain [-180, 180]
	 */
	public static double relativeAngle(double pivotAngle, double targetAngle) {
		double diff = targetAngle - pivotAngle;
		if (Math.abs(diff) <= 180.0)
			return diff;
		return diff - Math.signum(diff) * 360.0;
	}

	/**
	 * For systems where angles are given in the domain [-180, 180]: Returns the
	 * resultant angle from adding some delta angle to a pivot angle, accounting for
	 * the restriction that angles must be in this specific domain.
	 *
	 * @param pivotAngle
	 *            The angle that the deltaAngle is being added to (shifted)
	 * @param deltaAngle
	 *            The angle being added to pivot angle (the shift)
	 * @return The resultant angle of the shift in the domain [-180, 180]
	 */
	public static double shiftAngle(double pivotAngle, double deltaAngle) {
		return RobotMath.relativeAngle(-pivotAngle, deltaAngle);
	}

	/**
	 * Checking whether a number appoximates zero based on the threshholds set in
	 * Constants.RobotMathConstants
	 *
	 * @param value
	 *            The value to be checked
	 * @return True if the value approximates zero, false otherwise
	 */
	public static boolean approximatelyZero(double value) {
		return RobotMath.approxZero(value, RobotMathConstants.comparisonThreshhold);
	}

	/**
	 * Checking whether a number appoximates zero based on the threshhold given.
	 *
	 * @param value
	 *            The value to be checked
	 * @param threshhold
	 *            Determines how close the value needs to be to zero to pass
	 * @return True if the value approximates zero, false otherwise
	 */
	public static boolean approxZero(double value, double threshhold) {
		return value > -threshhold && value < threshhold;
	}

	/**
	 * Takes any angle in degrees in the domain (-infinity, infinitiy) and returns
	 * its value relative to 0 in the domain [-180, 180].
	 *
	 * @param angle
	 *            The absolute angle in the domain (-infinity, infinity)
	 * @return The angle's value relative to 0 in the domain [-180, 180]
	 */
	public static double constrain180(double angle) {
		return MathUtil.inputModulus(angle, -180, 180);
	}

	/**
	 * Takes an array of objects of generic type T and a function which converts
	 * such objects into a double value, and returns the sum of doing this
	 * conversion for each element in the array.
	 *
	 * @param <T>
	 *            The type being iterated over
	 * @param values
	 *            The values being iterated over and being given a corresponding
	 *            double value
	 * @param doubleConversion
	 *            The function that converts an object of type T into a double
	 * @return The sum of the result of applying 'doubleConversion' to all T objects
	 *         in 'values'
	 */
	public static <T> double sumForEach(T[] values, ToDoubleFunction<T> doubleConversion) {
		double sum = 0.0;
		for (T t : values)
			sum += doubleConversion.applyAsDouble(t);
		return sum;
	}

	/**
	 * Takes an array of objects of generic type T and a function which converts
	 * such objects into an int value, and returns the sum of doing this conversion
	 * for each element in the array.
	 *
	 * @param <T>
	 *            The type being iterated over
	 * @param values
	 *            The values being iterated over and being given a corresponding int
	 *            value
	 * @param intConversion
	 *            The function that converts an object of type T into an int
	 * @return The sum of the result of applying 'intConversion' to all T objects in
	 *         'values'
	 */
	public static <T> int sumForEach(T[] values, ToIntFunction<T> intConversion) {
		int sum = 0;
		for (T t : values)
			sum += intConversion.applyAsInt(t);
		return sum;
	}

	/**
	 * A classed used to handle running averages where past data should be discarded
	 * over time. This is often used for denoising purposes.
	 *
	 * As new data comes in, the previous data is reduced in weight by some factor,
	 * and over time, this makes old data approach irrelevancy.
	 */
	public static class DiminishingAverageHandler {

		private java.lang.Double average = null; // for nullability
		private final double previousWeight;

		/**
		 * Handles running averages where past data should be expelled over time. As new
		 * data comes in, the previous data is reduced in weight by some factor, and
		 * over time, this makes old data approach irrelevancy.
		 *
		 * @param previousWeight
		 *            The factor that previous data is multiplied by. This determines
		 *            how important past data is. The larger, the more important. The
		 *            closer to zero, the less important.
		 */
		public DiminishingAverageHandler(double previousWeight) {
			this.previousWeight = previousWeight;
		}

		/**
		 * Getter for the current average.
		 *
		 * @return The current average (null if still no data)
		 */
		public double get() {
			return this.average == null ? 0.0 : this.average;
		}

		/**
		 * Give the DiminishingAverageHandler new data and return the new average. If
		 * this is the first datum, then this will be returned as is.
		 *
		 * @param data
		 *            The new data
		 * @return The new average
		 */
		public double feed(double data) {
			if (average == null) {
				this.average = data;
				return this.average;
			}
			this.average = (this.average * this.previousWeight + data) / (1.0 + this.previousWeight);
			return average;
		}

		/**
		 * Resets the DimishingAverageHandler, deleting past data.
		 */
		public void reset() {
			this.average = null;
		}

	}
}
