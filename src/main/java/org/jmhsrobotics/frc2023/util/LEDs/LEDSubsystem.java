package org.jmhsrobotics.frc2023.util.LEDs;

import org.jmhsrobotics.frc2023.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

	public static enum LEDManager {

		/* ***** ADD NEW LED SECTIONS HERE ***** */
		STRIP0(65); // 65 lights

		public LEDBuffer buffer;

		private LEDManager(int numLights) {
			this.buffer = new LEDBuffer(numLights);
		}

	}

	private final int pwmPort = Constants.LEDConstants.LEDPWMPort;

	private BufferableAddressableLED driver;
	private LEDBuffer driverBuffer;

	public LEDSubsystem() {

		this.driver = new BufferableAddressableLED(this.pwmPort);

		int totalBufferSize = 0;
		for (var s : LEDManager.values())
			totalBufferSize += s.buffer.getLength();

		this.driverBuffer = new LEDBuffer(totalBufferSize);

		this.driver.setLength(this.driverBuffer.getLength());
		this.driver.start();
	}

	public void sendDriverBufferData() {
		this.driver.setData(this.driverBuffer);
	}

	public void sendData() {
		int acc = 0;
		for (var s : LEDManager.values()) {
			this.driverBuffer.copyAllFrom(s.buffer, acc);
			acc += s.buffer.getLength();
		}
		this.sendDriverBufferData();
	}

	public LEDBuffer getDriverBuffer() {
		return this.driverBuffer;
	}
}
