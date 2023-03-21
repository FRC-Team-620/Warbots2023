package org.jmhsrobotics.frc2023.util.LEDs;

public class LEDStripGroup extends LEDBuffer {

	private LEDStrip[] strips;

	public LEDStripGroup(LEDStrip... strips) {

		super(LEDStripGroup.totalBufferSize(strips));

		this.strips = strips;
	}

	private static int totalBufferSize(LEDStrip... strips) {
		int size = 0;
		for (LEDStrip s : strips)
			size += s.getLength();
		return size;
	}

	public void sendGroupData() {
		int acc = 0;
		for (LEDStrip s : this.strips) {
			for (int i = 0; i < s.getLength(); i++) {
				s.setLED(i, this.getLED(acc + i));
			}
			acc += s.getLength();
			s.sendData();
		}
	}

}
