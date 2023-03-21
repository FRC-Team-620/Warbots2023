// spotless:off

// package org.jmhsrobotics.frc2023.util.LEDs;

// public class LEDStripGroup extends LEDBuffer {

// 	private LEDSection[] strips;

// 	public LEDStripGroup(LEDSection... strips) {

// 		super(LEDStripGroup.totalBufferSize(strips));

// 		this.strips = strips;
// 	}

// 	private static int totalBufferSize(LEDSection... strips) {
// 		int size = 0;
// 		for (LEDSection s : strips)
// 			size += s.getLength();
// 		return size;
// 	}

// 	public void sendGroupData() {
// 		int acc = 0;
// 		for (LEDSection s : this.strips) {
// 			for (int i = 0; i < s.getLength(); i++) {
// 				s.setLED(i, this.getLED(acc + i));
// 			}
// 			acc += s.getLength();
// 			s.sendData();
// 		}
// 	}

// }

// spotless:on
