package org.jmhsrobotics.frc2023.util.LEDs;

import java.util.function.Consumer;

/**
 * This class handles an LED animation, which is a routine for the LEDs where
 * they change with time at some speed. The current state or frame of the
 * animation is sent to the LEDs when the step function is called on the
 * animation, which also moves the animation along one frame.
 */
public class LEDAnimation {

	private double accumulation, speed;
	private Consumer<Integer> animation;

	/**
	 * Creates an LEDAnimation object which handles a given animation at some speed.
	 *
	 * @param speed
	 *            The speed that the animation goes at. The closer to 0, the slower;
	 *            the larger, the faster. This changes how much the animation
	 *            changes when the step function is called.
	 * @param animation
	 *            This is an Integer Consumer object which takes in some number and
	 *            sets the corresponding animation state/frame to the LEDs.
	 */
	public LEDAnimation(double speed, Consumer<Integer> animation) {
		this.accumulation = 0.0;
		this.speed = speed;
		this.animation = animation;
	}

	/**
	 * Moves the animation along by an amount dictated by the LEDAnimation object's
	 * speed. This sets the LEDs to the current animation frame and then increments
	 * the animation accumulation (how far along the animation is).
	 */
	public void step() {
		this.animation.accept((int) this.accumulation);
		this.accumulation += this.speed;
	}

	/**
	 * Resets the accumulation, thus resetting the animation to the start
	 */
	public void reset() {
		this.accumulation = 0;
	}

	/**
	 * This static method combines any number of LEDAnimations and returns the
	 * animation that represents a concatenation of these animations. This new
	 * animation will play each animation for a number of frames dictated by the
	 * given speed, then move on to the next one, then the next, etc., and then wrap
	 * around to the first. Each animation will accumulate when shown.
	 *
	 * @param speed
	 *            The frequency of animation switches. The closer to 0, the longer
	 *            each animation will show for; the closer to one, the faster; if
	 *            higher than one, some animations will be skipped or passed over on
	 *            some passes, depending on the speed and how many animations are
	 *            given. For example, if the speed is 0.1, it will take 10 calls to
	 *            step for the next animation to show, if it is 1, then a new
	 *            animation will show each frame.
	 * @param animations
	 *            The animations to be rotated between, in order.
	 * @return The animation corresponding to the concatenation of the given
	 *         animations.
	 */
	public static LEDAnimation concatenate(double speed, LEDAnimation... animations) {
		return new LEDAnimation(speed, n -> animations[n % animations.length].step());
	}
}
