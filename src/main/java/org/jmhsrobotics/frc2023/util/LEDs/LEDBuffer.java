package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

public class LEDBuffer {

	private byte[] buffer;

	public LEDBuffer(int numLights) {
		buffer = new byte[numLights * 4];
	}

	/**
	 * *** Taken from wpilib 'AddressableLEDBuffer' ***
	 *
	 * <p>
	 * Sets a specific led in the buffer.
	 *
	 * @param index
	 *            the index to write
	 * @param r
	 *            the r value [0-255]
	 * @param g
	 *            the g value [0-255]
	 * @param b
	 *            the b value [0-255]
	 */
	public void setRGB(int index, int r, int g, int b) {
		this.buffer[index * 4] = (byte) b;
		this.buffer[(index * 4) + 1] = (byte) g;
		this.buffer[(index * 4) + 2] = (byte) r;
		this.buffer[(index * 4) + 3] = 0;
	}

	/**
	 * *** Taken from wpilib 'AddressableLEDBuffer' ***
	 *
	 * <p>
	 * Sets a specific LED in the buffer.
	 *
	 * @param index
	 *            The index to write
	 * @param color
	 *            The color of the LED
	 */
	public void setLED(int index, Color color) {
		this.setRGB(index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
	}

	/**
	 * *** Taken from wpilib 'AddressableLEDBuffer' ***
	 *
	 * <p>
	 * Gets the color at the specified index.
	 *
	 * @param index
	 *            the index to get
	 * @return the LED color at the specified index
	 */
	public Color getLED(int index) {
		// spotless:off
		return new Color(
			(this.buffer[index * 4 + 2] & 0xFF) / 255.0, 
			(this.buffer[index * 4 + 1] & 0xFF) / 255.0,
			(this.buffer[index * 4] & 0xFF) / 255.0
		);
		// spotless:on
	}

	/**
	 * *** Taken from wpilib 'AddressableLEDBuffer' ***
	 *
	 * <p>
	 * Gets the buffer length (the LED count).
	 *
	 * @return the buffer length
	 */
	public int getLength() {
		return this.buffer.length / 4;
	}

	public void copyFrom(LEDBuffer other, int targetStartIndex, int sourceStartIndex, int numLights) {
		System.arraycopy(other.buffer, sourceStartIndex * 4, this.buffer, targetStartIndex * 4, numLights * 4);
	}

	public void copyAllFrom(LEDBuffer other, int targetStartIndex) {
		this.copyFrom(other, targetStartIndex, 0, other.getLength());
	}

	public void fillFrom(LEDBuffer other, int sourceStartIndex) {
		this.copyFrom(other, 0, sourceStartIndex, this.getLength());
	}

	public byte[] getBufferData() {
		return this.buffer;
	}

	// ************* STATIC COLOR UTIL METHODS *************

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
			gradient[i] = LEDBuffer.colorInterpolation(proportion, startColor, endColor);
		}
		return gradient;
	}

	// ************* SETTING BUFFER PATTERNS *************

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
		int lightsPerGrad = this.getLength() / colors.length;
		int finalGradPos = lightsPerGrad * colors.length - 1;
		for (int i = 0; i < colors.length; i++) {
			grad = LEDBuffer.colorGradient(colors[i], colors[(i + 1) % colors.length], lightsPerGrad);
			for (int k = 0; k < lightsPerGrad; k++) {
				int lightIdx = i * lightsPerGrad + k;
				this.setLED((lightIdx + offset) % (finalGradPos + 1), grad[k]);
			}
		}

		// Correcting error resulting from integer division
		Color finalColor = this.getLED(finalGradPos);
		for (int i = finalGradPos; i < this.getLength(); i++)
			this.setLED(i, finalColor);
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
			this.setLED(i, color);
		}
	}

	/**
	 * Sets the entire LED strip to a given color.
	 *
	 * @param color
	 *            The color to be set to the LED strip.
	 */
	public void setSolidColor(Color color) {
		this.setSubsetSolidColor(0, this.getLength(), color);
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
		int idx = 0, bufferSize = this.getLength();
		for (int i = 0; idx < bufferSize; i++) {
			for (int k = 0; k < lengths[i % lengths.length] && idx + k < bufferSize; k++)
				this.setLED((idx + k + offset) % bufferSize, colors[i % colors.length]);
			idx += lengths[i % lengths.length];
		}
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

	// ************* ANIMATIONS *************

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
		// spotless:off
        return new LEDAnimation(speed, n -> {
            n %= (bidirectional ? 2 * steps : steps);
            int i = bidirectional ? -Math.abs(n - steps) + steps : n;
            double proportion = (double) i / steps;
            this.setSolidColor(new Color(
                proportion * (c2.red - c1.red) + c1.red,
                proportion * (c2.green - c1.green) + c1.green, 
                proportion * (c2.blue - c1.blue) + c1.blue
            ));
        });
        // spotless:on
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

}
