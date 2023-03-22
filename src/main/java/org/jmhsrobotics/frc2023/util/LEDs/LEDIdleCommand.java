package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.SectionManager;

public class LEDIdleCommand extends CommandBase {

	BooleanSupplier condition, sidebandCondition;

	private LEDAnimation yellowGradAnim = SectionManager.BODY.buffer.gradientAnimation(1, Color.kYellow,
			Color.kOrangeRed);
	private LEDAnimation blueGradAnim = SectionManager.BODY.buffer.gradientAnimation(1, Color.kBlue, Color.kBlueViolet,
			Color.kPurple);

	// spotless:off
	private LEDSectionGroup sidebarGroup = new LEDSectionGroup(
		SectionManager.LOWBAND,
		SectionManager.HIGHBAND
	);
	// spotless:on

	private LEDAnimation sidebarGreenBlock = sidebarGroup.colorBlockAnimation(0.2, new int[]{5, 5}, Color.kGreen,
			Color.kGreenYellow);

	private LEDAnimation sidebarRedBlock = sidebarGroup.colorBlockAnimation(0.2, new int[]{5, 5}, Color.kRed,
			Color.kMediumVioletRed);

	// private LEDAnimation highGreenBlock =
	// SectionManager.HIGHBAND.buffer.colorBlockAnimation(0.2, new int[]{5, 5},
	// Color.kGreen, Color.kGreenYellow);
	// private LEDAnimation highRedBlock =
	// SectionManager.HIGHBAND.buffer.colorBlockAnimation(0.2, new int[]{5, 5},
	// Color.kRed, Color.kMediumVioletRed);

	// private LEDAnimation lowGreenBlock =
	// SectionManager.LOWBAND.buffer.colorBlockAnimation(0.2, new int[]{5, 5},
	// Color.kGreen, Color.kGreenYellow);
	// private LEDAnimation lowRedBlock =
	// SectionManager.LOWBAND.buffer.colorBlockAnimation(0.2, new int[]{5, 5},
	// Color.kRed, Color.kMediumVioletRed);

	LEDSubsystem ledSubsystem;

	public LEDIdleCommand(LEDSubsystem ledSubsystem, BooleanSupplier condition, BooleanSupplier sidebandCondition) {

		this.ledSubsystem = ledSubsystem;

		this.condition = condition;
		this.sidebandCondition = sidebandCondition;

		this.addRequirements(this.ledSubsystem);
	}

	@Override
	public void execute() {

		if (this.condition != null) {
			if (this.condition.getAsBoolean()) {
				this.yellowGradAnim.step(); // RED if true
			} else {
				this.blueGradAnim.step(); // BLUE if false
			}
			SectionManager.BODY.buffer.setSubsetSolidColor(0, 2, Color.kBlack);
			SectionManager.BODY.buffer.setSubsetSolidColor(43, 45, Color.kBlack);
		}

		if (this.sidebandCondition != null) {
			if (this.sidebandCondition.getAsBoolean()) {
				this.sidebarRedBlock.step();
			} else {
				this.sidebarGreenBlock.step();
			}
			this.sidebarGroup.writeToSections();
		}

		// this.strip.sendData();
		this.ledSubsystem.sendSectionData();
	}
}
