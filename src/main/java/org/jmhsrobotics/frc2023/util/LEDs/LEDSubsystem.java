package org.jmhsrobotics.frc2023.util.LEDs;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.RobotMath;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

	public static enum SectionManager {

		/* ***** ADD NEW LED SECTIONS HERE ***** */
		LOWBAND(10), BODY(45), HIGHBAND(10);

		public LEDBuffer buffer;

		private SectionManager(int numLights) {
			this.buffer = new LEDBuffer(numLights);
		}

		/**
		 * Gets the total buffer length represented by the given LED sections in the
		 * SectionManager.
		 *
		 * @param sections
		 *            The LED sections from the SectionManager
		 * @return Their combined buffer length
		 */
		public static int totalBufferLength(SectionManager... sections) {
			return RobotMath.sumForEach(sections, (SectionManager s) -> s.buffer.getLength());
		}
	}

	private final int pwmPort = Constants.LEDConstants.LEDPWMPort;
	public final int driverBufferLength = SectionManager.totalBufferLength(SectionManager.values());

	private BufferableAddressableLED driver;
	private LEDBuffer driverBuffer;

	public LEDSubsystem() {

		this.driver = new BufferableAddressableLED(this.pwmPort);

		this.driverBuffer = new LEDBuffer(this.driverBufferLength);

		this.driver.setLength(this.driverBufferLength);
		this.driver.start();
	}

	/**
	 * Copies the LED values stored in each LED section in the SectionManager to the
	 * main driver buffer (the buffer which is actually written to the LED
	 * strip/series of strips) in their appropriate locations.
	 *
	 * <p>
	 * Note: This does not need to be called if you are using the 'sendSectionData'
	 * method to write to the LEDs, as this method does this automatically.
	 */
	public void writeSectionsToDriverBuffer() {
		int acc = 0;
		for (var s : SectionManager.values()) {
			this.driverBuffer.copyAllFrom(s.buffer, acc);
			acc += s.buffer.getLength();
		}
	}

	/**
	 * Sends the LED data stored in the main driver buffer (the buffer which is
	 * actually written to the LED strip/series of strips) to the LEDs.
	 *
	 * <p>
	 * Note: Only one method which sends data to the LEDs (viz.
	 * 'sendDriverBufferData' and 'sendSectionData') should be called per tick.
	 */
	public void sendDriverBufferData() {
		this.driver.setData(this.driverBuffer);
	}

	/**
	 * Copies the LED values stored in each LED section in the SectionManager to the
	 * main driver buffer (the buffer which is actually written to the LED
	 * strip/series of strips) in their appropriate locations, and then sends the
	 * LED data stored in the main driver buffer to the LEDs.
	 *
	 * <p>
	 * Note: Only one method which sends data to the LEDs (viz.
	 * 'sendDriverBufferData' and 'sendSectionData') should be called per tick.
	 */
	public void sendSectionData() {
		this.writeSectionsToDriverBuffer();
		this.sendDriverBufferData();
	}

	/**
	 * Gets the main driver buffer (the buffer which is actually written to the LED
	 * strip/series of strips) so that all the LEDs can be written to at once.
	 *
	 * <p>
	 * Note: Any animations that are created based on this buffer will act as though
	 * all of the LEDs on the robot constitute one LED section, and will loop
	 * through their entirety, which could be desirable.
	 *
	 * @return The driver buffer
	 */
	public LEDBuffer getDriverBuffer() {
		return this.driverBuffer;
	}
}
