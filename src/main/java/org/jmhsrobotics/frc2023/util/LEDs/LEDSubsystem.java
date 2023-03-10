package org.jmhsrobotics.frc2023.util.LEDs;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

	public static enum LEDManager {

		/* ***** ADD NEW LED STRIPS HERE ***** */
		STRIP0(65, 9); // 65 lights, PWM port 9

		public LEDStrip strip;

		private LEDManager(int LEDCount, int PWMPort) {
			this.strip = new LEDStrip(LEDCount, PWMPort);
		}

	}

	/**
	 * This class handles an LED animation, which is a routine for the LEDs where
	 * they change with time at some speed. The current state or frame of the
	 * animation is sent to the LEDs when the step function is called on the
	 * animation, which also moves the animation along one frame.
	 */
	public static class LEDAnimation {

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

	/**
	 * Manages an addressable LED strip at some PWMPort with a given count of LEDs.
	 */
	public static class LEDStrip {

		private int PWMPort;
		private AddressableLED lights;
		private AddressableLEDBuffer buffer;

		/**
		 * Creates an LEDStrip object to manage an addressable LED strip at a given PWM
		 * port with a given LED count.
		 *
		 * @param numLights
		 *            How many LEDs are on the strip, or how many LEDs on a strip should
		 *            be addressed.
		 * @param PWMPort
		 *            The PWM port that this LED strip is connected to.
		 */
		public LEDStrip(int numLights, int PWMPort) {

			this.PWMPort = PWMPort;

			// Must be a PWM header
			this.lights = new AddressableLED(this.PWMPort);

			// Reuse buffer
			// Length is expensive to set, so only set it once, then just update data
			this.buffer = new AddressableLEDBuffer(numLights);
			this.lights.setLength(this.buffer.getLength());

			// Set the data
			this.lights.setData(this.buffer);
			this.lights.start();
		}

		/**
		 * Gets the length of the LED strip (number of LEDs).
		 * 
		 * @return The number of LEDs
		 */
		public int getLength() {
			return this.buffer.getLength();
		}

		/**
		 * Creates an animation which creates a gradient of an arbitrary number of
		 * colors and moves it along the LED strip.
		 *
		 * @param speed
		 *            The speed of the animation (how much each call to step moves the
		 *            animation along).
		 * @param colors
		 *            The colors in the gradient.
		 * @return The animation corresponding to a gradient of these colors being moved
		 *         along the strip.
		 */
		public LEDAnimation gradientAnimation(double speed, Color... colors) {
			return new LEDAnimation(speed, n -> this.setGradient(n, colors));
		}

		/**
		 * Creates an animation which creates a series of blocks of color of a given
		 * length pattern and a given color pattern. The length and color pattern are
		 * cycled down the length of the LED strip separately, which allows for complex
		 * patterns. For example, the blocks might be set to be of size, 1, 2, and 3,
		 * repeating, and then the colors might be set to be red and blue (alternating
		 * between the two). The animation moves this pattern along the LED strip.
		 *
		 * @param speed
		 *            The speed of the animation (how much each call to step moves the
		 *            animation along).
		 * @param lengthPattern
		 *            The number of LEDs corresponding to each color block in the
		 *            pattern. This pattern is repeated along the LED strip.
		 * @param colors
		 *            The colors of the LEDs corresponding to each color block in the
		 *            pattern. This pattern is repeated along the LED strip.
		 * @return The animation corresponding to this pattern moving along the LED
		 *         strip at some speed.
		 */
		public LEDAnimation colorBlockAnimation(double speed, int[] lengthPattern, Color... colors) {
			return new LEDAnimation(speed, n -> this.setColorBlocks(n, lengthPattern, colors));
		}

		/**
		 * Creates an animation which cycles between solid colors at a certain speed.
		 *
		 * @param speed
		 *            The frequency of color switches. The closer to 0, the longer each
		 *            color will show for; the closer to one, the faster; if higher than
		 *            one, some colors will be skipped or passed over on some passes,
		 *            depending on the speed and how many colors are given. For example,
		 *            if the speed is 0.1, it will take 10 calls to step for the next
		 *            color to show, if it is 1, then a new color will show each frame.
		 * @param colors
		 *            The colors to be rotated between, in order.
		 * @return The animation corresponding to these colors being cycled between at a
		 *         given frequency.
		 */
		public LEDAnimation blinkingAnimation(double speed, Color... colors) {
			return new LEDAnimation(speed, n -> this.setSolidColor(colors[n % colors.length]));
		}

		/**
		 * Creates an animation which sets the LEDs to a solid color. When the step
		 * function is called, there is no change to the LEDs. If another animation is
		 * currently shown on the LEDs, calling the step function of this animation will
		 * set the LEDs to be a given solid color.
		 *
		 * @param color
		 *            The color to set the LEDs to.
		 * @return An animation corresponding to setting the LEDs to a solid color.
		 */
		public LEDAnimation solidColorAnimation(Color color) {
			return new LEDAnimation(0, n -> this.setSolidColor(color));
		}

		/**
		 * Creates an animation which fades between two solid colors.
		 *
		 * @param speed
		 *            The speed of the animation (how much each call to step moves the
		 *            animation along).
		 * @param steps
		 *            How many steps are in the fading gradient.
		 * @param bidirectional
		 *            Whether the second color should gradually return to the first
		 *            color after the first fade occurs (true), or cut back to the first
		 *            and repeat the fade from the first to the second color (false).
		 * @param c1
		 *            The first color in the fade animation.
		 * @param c2
		 *            The second color in the fade animation.
		 * @return The animation corresponding to a fade between two colors.
		 */
		public LEDAnimation fadeTwoAnimation(double speed, int steps, boolean bidirectional, Color c1, Color c2) {
			return new LEDAnimation(speed, n -> {
				n %= (bidirectional ? 2 * steps : steps);
				int i = bidirectional ? -Math.abs(n - steps) + steps : n;
				double proportion = (double) i / steps;
				this.setSolidColor(new Color(proportion * (c2.red - c1.red) + c1.red,
						proportion * (c2.green - c1.green) + c1.green, proportion * (c2.blue - c1.blue) + c1.blue));
			});
		}

		/**
		 * Creates an animation which fades between two solid colors in both directions.
		 *
		 * @param speed
		 *            The speed of the animation (how much each call to step moves the
		 *            animation along).
		 * @param steps
		 *            How many steps are in the fading gradient.
		 * @param c1
		 *            The first color in the fade animation.
		 * @param c2
		 *            The second color in the fade animation.
		 * @return The animation corresponding to a bidirectional fade between two
		 *         colors.
		 */
		public LEDAnimation fadeTwoAnimation(double speed, int steps, Color c1, Color c2) {
			return this.fadeTwoAnimation(speed, steps, true, c1, c2);
		}

		/**
		 * Creates an animation which fades between a number of solid colors in both
		 * directions in a given order.
		 *
		 * @param speed
		 *            The speed of the animation (how much each call to step moves the
		 *            animation along).
		 * @param stepsPer
		 *            How many steps are in the fading gradient.
		 * @param colors
		 *            The colors in the fade animation to be faded between, in order.
		 * @return The animation corresponding to fading between the given colors.
		 */
		public LEDAnimation fadeAnimation(double speed, int stepsPer, Color... colors) {
			// Edge cases
			if (colors.length <= 0)
				return null;
			else if (colors.length == 1)
				return this.solidColorAnimation(colors[0]);
			else if (colors.length == 2)
				return this.fadeTwoAnimation(speed, stepsPer, true, colors[0], colors[1]);
			// Concatenating a series of fade animations
			LEDAnimation[] fades = new LEDAnimation[colors.length];
			for (int i = 0; i < colors.length; i++)
				fades[i] = this.fadeTwoAnimation(speed, stepsPer, false, colors[i], colors[(i + 1) % colors.length]);
			return LEDAnimation.concatenate(speed / stepsPer, fades);
		}

		/**
		 * Returns an RGB representation of the light at a given index along the LED
		 * strip.
		 *
		 * @param index
		 *            The index of the LED.
		 * @return An RGB representation of the LED at this index.
		 */
		public Color colorAt(int index) {
			return this.buffer.getLED(index);
		}

		/**
		 * Sets the LED at a given index to a given color.
		 *
		 * @param index
		 *            The index of the LED.
		 * @param color
		 *            The color to set the LED to.
		 */
		public void set(int index, Color color) {
			this.buffer.setLED(index, color);
			this.lights.setData(this.buffer);
		}

		/**
		 * Sets the LEDs in the range [startIndex, endIndex) to the given color. Does
		 * not touch other LEDs.
		 *
		 * @param startIndex
		 *            The index of the first LED in the range (inclusive).
		 * @param endIndex
		 *            The index of the last LED in the range (exclusive).
		 * @param color
		 *            The color to set the LEDs to.
		 */
		public void setSubsetSolidColor(int startIndex, int endIndex, Color color) {
			for (int i = startIndex; i < endIndex; i++) {
				this.buffer.setLED(i, color);
			}
			this.lights.setData(this.buffer);
		}

		/**
		 * Sets the entire LED strip to a given color.
		 *
		 * @param color
		 *            The color to be set to the LED strip.
		 */
		public void setSolidColor(Color color) {
			this.setSubsetSolidColor(0, this.buffer.getLength(), color);
		}

		/**
		 * Sets the LED strip to a gradient melding between a number of colors.
		 *
		 * @param colors
		 *            The colors that make up the gradient.
		 */
		public void setGradient(Color... colors) {
			this.setGradient(0, colors);
		}

		/**
		 * Sets the LED strip to a gradient melding between a number of colors.
		 *
		 * @param offset
		 *            The offset from the default position that the gradient starts at.
		 * @param colors
		 *            The colors that make up the gradient.
		 */
		public void setGradient(int offset, Color... colors) {
			Color grad[];
			int lightsPerGrad = this.buffer.getLength() / colors.length;
			int finalGradPos = lightsPerGrad * colors.length - 1;
			for (int i = 0; i < colors.length; i++) {
				grad = LEDStrip.colorGradient(colors[i], colors[(i + 1) % colors.length], lightsPerGrad);
				for (int k = 0; k < lightsPerGrad; k++) {
					int lightIdx = i * lightsPerGrad + k;
					this.buffer.setLED((lightIdx + offset) % (finalGradPos + 1), grad[k]);
				}
			}

			// Correcting error resulting from integer division
			Color finalColor = this.colorAt(finalGradPos);
			for (int i = finalGradPos; i < this.buffer.getLength(); i++)
				this.buffer.setLED(i, finalColor);

			// Setting the data
			this.lights.setData(this.buffer);
		}

		/**
		 * Sets the LED strip to a series of blocks of color of a given length pattern
		 * and a given color pattern. The length and color pattern are cycled down the
		 * length of the LED strip separately, which allows for complex patterns. For
		 * example, the blocks might be set to be of size, 1, 2, and 3, repeating, and
		 * then the colors might be set to be red and blue (alternating between the
		 * two).
		 *
		 * @param lengths
		 *            The number of LEDs corresponding to each color block in the
		 *            pattern. This pattern is repeated along the LED strip.
		 * @param colors
		 *            The colors of the LEDs corresponding to each color block in the
		 *            pattern. This pattern is repeated along the LED strip.
		 */
		public void setColorBlocks(int[] lengths, Color... colors) {
			this.setColorBlocks(0, lengths, colors);
		}

		/**
		 * Sets the LED strip to a series of blocks of color of a given length pattern
		 * and a given color pattern. The length and color pattern are cycled down the
		 * length of the LED strip separately, which allows for complex patterns. For
		 * example, the blocks might be set to be of size, 1, 2, and 3, repeating, and
		 * then the colors might be set to be red and blue (alternating between the
		 * two).
		 *
		 * @param offset
		 *            The offset from the default position that the gradient starts at.
		 * @param lengths
		 *            The number of LEDs corresponding to each color block in the
		 *            pattern. This pattern is repeated along the LED strip.
		 * @param colors
		 *            The colors of the LEDs corresponding to each color block in the
		 *            pattern. This pattern is repeated along the LED strip.
		 */
		public void setColorBlocks(int offset, int[] lengths, Color... colors) {
			int idx = 0, bufferSize = this.buffer.getLength();
			for (int i = 0; idx < bufferSize; i++) {
				for (int k = 0; k < lengths[i % lengths.length] && idx + k < bufferSize; k++)
					this.buffer.setLED((idx + k + offset) % bufferSize, colors[i % colors.length]);
				idx += lengths[i % lengths.length];
			}
			this.lights.setData(this.buffer);
		}

		/**
		 * Given two colors, this returns a color that is a combination of the two, with
		 * some proportion representing how much of the first color to put in over the
		 * other.
		 *
		 * @param proportion
		 *            How much of color1 (0 - 1) to use in the
		 *            combination/interpolation, i.e. how much weight to give to color1
		 *            as opposed to color 2.
		 * @param color1
		 *            The first color to be combined.
		 * @param color2
		 *            The second color to be combined.
		 * @return The weighted combination of the two colors.
		 */
		public static Color colorInterpolation(double proportion, Color color1, Color color2) {
			proportion = MathUtil.clamp(proportion, 0, 1);
			// spotless:off
			return new Color(
					proportion * (color2.red - color1.red) + color1.red,
					proportion * (color2.green - color1.green) + color1.green,
					proportion * (color2.blue - color1.blue) + color1.blue
				);
			// spotless:on
		}

		/**
		 * Creates an array of RGB colors representing a gradient between two given
		 * colors over a number of steps.
		 *
		 * @param startColor
		 *            The color to start the gradient at.
		 * @param endColor
		 *            The color to end the gradient at.
		 * @param steps
		 *            The number of steps/individual colors in the gradient, i.e. the
		 *            smoothness/length of the gradient.
		 * @return An array of RGB colors representing a gradient between the two given
		 *         colors.
		 */
		private static Color[] colorGradient(Color startColor, Color endColor, int steps) {
			Color gradient[] = new Color[steps]; // the colors in the gradient
			double proportion;
			for (int i = 0; i < gradient.length; i++) {
				proportion = (double) i / (gradient.length - 1); // how far along in the gradient
				gradient[i] = LEDStrip.colorInterpolation(proportion, startColor, endColor);
			}
			return gradient;
		}
	}

	/**
	 * A dummy constructor. This is necessary in order to create commands around the
	 * LEDs and to ensure that multiple commands are not run at once that set the
	 * LEDs (addRequirements)
	 */
	public LEDSubsystem() {
	}
}
