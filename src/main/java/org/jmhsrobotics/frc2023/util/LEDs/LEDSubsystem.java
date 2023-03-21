package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

	public static enum LEDManager {

		/* ***** ADD NEW LED STRIPS HERE ***** */
		STRIP0(65, 9); // 65 lights, PWM port 9

		public LEDStrip strip;

		private LEDManager(int LEDCount, int PWMPort) {
			this.strip = new LEDStrip(LEDCount, PWMPort);
		}

	}

	/**
	 * A dummy constructor. This is necessary in order to create commands around the
	 * LEDs and to ensure that multiple commands are not run at once that set the
	 * LEDs (addRequirements)
	 */
	public LEDSubsystem() {
	}
}
