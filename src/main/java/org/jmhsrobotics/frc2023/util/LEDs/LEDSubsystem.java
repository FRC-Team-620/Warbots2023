package org.jmhsrobotics.frc2023.util.LEDs;

import org.jmhsrobotics.frc2023.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

	public static enum SectionManager {

		/* ***** ADD NEW LED SECTIONS HERE ***** */
		LOWBAND(10), BODY(45), HIGHBAND(10);

		public LEDBuffer buffer;

		private SectionManager(int numLights) {
			this.buffer = new LEDBuffer(numLights);
		}

		public static int totalBufferLength(SectionManager... sections) {
			int size = 0;
			for (var s : sections)
				size += s.buffer.getLength();
			return size;
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

	public void writeSectionsToDriverBuffer() {
		int acc = 0;
		for (var s : SectionManager.values()) {
			this.driverBuffer.copyAllFrom(s.buffer, acc);
			acc += s.buffer.getLength();
		}
	}

	public void sendDriverBufferData() {
		this.driver.setData(this.driverBuffer);
	}

	public void sendSectionData() {
		this.writeSectionsToDriverBuffer();
		this.sendDriverBufferData();
	}

	public LEDBuffer getDriverBuffer() {
		return this.driverBuffer;
	}
}
