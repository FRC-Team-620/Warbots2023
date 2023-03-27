package org.jmhsrobotics.frc2023.util.LEDs;

import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.SectionManager;

/**
 * Combines a number of LED sections from the SectionManager such that they can
 * be written to as though they were one section, even if they are far apart
 * from one another. This allows, for example, animations to loop through them
 * as though they were one larger or combined section.
 */
public class LEDSectionGroup extends LEDBuffer {

	private SectionManager[] sections;

	/**
	 * Creates an LEDSectionGroup combining the given LED sections from the
	 * SectionManager.
	 *
	 * @param sections
	 *            The LED sections to be combined
	 */
	public LEDSectionGroup(SectionManager... sections) {

		super(SectionManager.totalBufferLength(sections));

		this.sections = sections;
	}

	/**
	 * Copies the data stored in this LEDSectionGroup object's buffer to the
	 * corresponding constituent LED sections of the SectionManager.
	 */
	public void writeToSections() {
		int acc = 0;
		for (var s : this.sections) {
			s.buffer.fillFrom(this, acc);
			acc += s.buffer.getLength();
		}
	}
}
