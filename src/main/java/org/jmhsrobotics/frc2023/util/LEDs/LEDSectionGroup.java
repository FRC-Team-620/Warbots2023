package org.jmhsrobotics.frc2023.util.LEDs;

import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.SectionManager;

public class LEDSectionGroup extends LEDBuffer {

	private SectionManager[] sections;

	public LEDSectionGroup(SectionManager... sections) {

		super(SectionManager.totalBufferLength(sections));

		this.sections = sections;
	}

	public void writeToSections() {
		int acc = 0;
		for (var s : this.sections) {
			s.buffer.fillFrom(this, acc);
			acc += s.buffer.getLength();
		}
	}
}
