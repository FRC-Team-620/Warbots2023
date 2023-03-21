// spotless:off


// package org.jmhsrobotics.frc2023.util.LEDs;

// /**
//  * Manages an addressable LED strip at some PWMPort with a given count of LEDs.
//  */
// public class LEDSection extends LEDBuffer {

// 	private int PWMPort;

// 	/**
// 	 * Creates an LEDStrip object to manage an addressable LED strip at a given PWM
// 	 * port with a given LED count.
// 	 *
// 	 * @param numLights
// 	 *            How many LEDs are on the strip, or how many LEDs on a strip should
// 	 *            be addressed.
// 	 * @param PWMPort
// 	 *            The PWM port that this LED strip is connected to.
// 	 */
// 	public LEDSection(int numLights) {

// 		super(numLights);

// 		// this.PWMPort = PWMPort;

// 		// // Must be a PWM header
// 		// this.lights = new BufferableAddressableLED(this.PWMPort);

// 		// // Reuse buffer
// 		// // Length is expensive to set, so only set it once, then just update data
// 		// this.buffer = new LEDBuffer(numLights);
// 		// this.lights.setLength(this.buffer.getLength());

// 		// Reuse buffer
// 		// Length is expensive to set, so only set it once, then just update data
// 		// this.lights.setLength(this.getLength());

// 		// Set the data
// 		this.sendData();
// 		// this.lights.start();
// 	}

// 	/**
// 	 * This sends the changes that have been made to the LED strip to be rendered on
// 	 * the LEDs. This should only be called ONCE per frame, and MUST be called in
// 	 * order for anything to be rendered.
// 	 */
// 	public void sendData() {
// 		this.lights.setData(this);
// 	}

	
// 	public static int totalBufferSize(LEDSection... strips) {
// 		int size = 0;
// 		for (LEDSection s : strips)
// 			size += s.getLength();
// 		return size;
// 	}

// }

// spotless:on
