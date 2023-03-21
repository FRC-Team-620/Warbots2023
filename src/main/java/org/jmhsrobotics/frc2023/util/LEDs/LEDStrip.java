package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;

/**
 * Manages an addressable LED strip at some PWMPort with a given count of LEDs.
 */
public class LEDStrip extends LEDBuffer {

	private int PWMPort;
	private AddressableLED lights;

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

		super(numLights);

		this.PWMPort = PWMPort;

		// Must be a PWM header
		this.lights = new AddressableLED(this.PWMPort);

		// // Reuse buffer
		// // Length is expensive to set, so only set it once, then just update data
		// this.buffer = new LEDBuffer(numLights);
		// this.lights.setLength(this.buffer.getLength());

		// Reuse buffer
		// Length is expensive to set, so only set it once, then just update data
		this.lights.setLength(this.getLength());

		// Set the data
		this.sendData();
		this.lights.start();
	}

	/**
	 * This sends the changes that have been made to the LED strip to be rendered on
	 * the LEDs. This should only be called ONCE per frame, and MUST be called in
	 * order for anything to be rendered.
	 */
	public void sendData() {
		this.lights.setData(this);
	}

}
