package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PWMJNI;

/**
 * *** Mostly taken from wpilib 'AddressableLED' ***
 *
 * <p>
 * A class for driving addressable LEDs, such as WS2812s and NeoPixels.
 *
 * <p>
 * Only 1 LED driver is currently supported by the roboRIO.
 */
public class BufferableAddressableLED implements AutoCloseable {
	private final int pwmHandle;
	private final int handle;

	/**
	 * *** Taken from wpilib 'AddressableLED' ***
	 *
	 * <p>
	 * Constructs a new driver for a specific port.
	 *
	 * @param port
	 *            the output port to use (Must be a PWM header, not on MXP)
	 */
	public BufferableAddressableLED(int port) {
		this.pwmHandle = PWMJNI.initializePWMPort(HAL.getPort((byte) port));
		this.handle = AddressableLEDJNI.initialize(this.pwmHandle);
		HAL.report(tResourceType.kResourceType_AddressableLEDs, port + 1);
	}

	@Override
	public void close() {
		if (this.handle != 0) {
			AddressableLEDJNI.free(this.handle);
		}
		if (this.pwmHandle != 0) {
			PWMJNI.freePWMPort(this.pwmHandle);
		}
	}

	/**
	 * *** Taken from wpilib 'AddressableLED' ***
	 *
	 * <p>
	 * Sets the length of the LED strip.
	 *
	 * <p>
	 * Calling this is an expensive call, so it's best to call it once, then just
	 * update data.
	 *
	 * <p>
	 * The max length is 5460 LEDs.
	 *
	 * @param length
	 *            the strip length
	 */
	public void setLength(int length) {
		AddressableLEDJNI.setLength(this.handle, length);
	}

	/**
	 * Sets the LED output data.
	 *
	 * <p>
	 * If the output is enabled, this will start writing the next data cycle. It is
	 * safe to call, even while output is enabled.
	 *
	 * @param buffer
	 *            the buffer to write
	 */
	public void setData(LEDBuffer buffer) {
		AddressableLEDJNI.setData(this.handle, buffer.getBufferData());
	}

	/**
	 * *** Taken from wpilib 'AddressableLED' ***
	 *
	 * <p>
	 * Sets the bit timing.
	 *
	 * <p>
	 * By default, the driver is set up to drive WS2812s, so nothing needs to be set
	 * for those.
	 *
	 * @param lowTime0NanoSeconds
	 *            low time for 0 bit
	 * @param highTime0NanoSeconds
	 *            high time for 0 bit
	 * @param lowTime1NanoSeconds
	 *            low time for 1 bit
	 * @param highTime1NanoSeconds
	 *            high time for 1 bit
	 */
	public void setBitTiming(int lowTime0NanoSeconds, int highTime0NanoSeconds, int lowTime1NanoSeconds,
			int highTime1NanoSeconds) {
		AddressableLEDJNI.setBitTiming(this.handle, lowTime0NanoSeconds, highTime0NanoSeconds, lowTime1NanoSeconds,
				highTime1NanoSeconds);
	}

	/**
	 * *** Taken from wpilib 'AddressableLED' ***
	 *
	 * <P>
	 * Sets the sync time.
	 *
	 * <p>
	 * The sync time is the time to hold output so LEDs enable. Default set for
	 * WS2812.
	 *
	 * @param syncTimeMicroSeconds
	 *            the sync time
	 */
	public void setSyncTime(int syncTimeMicroSeconds) {
		AddressableLEDJNI.setSyncTime(this.handle, syncTimeMicroSeconds);
	}

	/**
	 * *** Taken from wpilib 'AddressableLED' ***
	 *
	 * <p>
	 * Starts the output.
	 *
	 * <p>
	 * The output writes continuously.
	 */
	public void start() {
		AddressableLEDJNI.start(this.handle);
	}

	/**
	 * *** Taken from wpilib 'AddressableLED' ***
	 *
	 * <p>
	 * Stops the output.
	 */
	public void stop() {
		AddressableLEDJNI.stop(this.handle);
	}
}
