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

	private LEDAnimation highGreenBlock = SectionManager.HIGHBAND.buffer.colorBlockAnimation(0.2, new int[]{5, 5},
			Color.kGreen, Color.kGreenYellow);
	private LEDAnimation highRedBlock = SectionManager.HIGHBAND.buffer.colorBlockAnimation(0.2, new int[]{5, 5},
			Color.kRed, Color.kMediumVioletRed);

	private LEDAnimation lowGreenBlock = SectionManager.LOWBAND.buffer.colorBlockAnimation(0.2, new int[]{5, 5},
			Color.kGreen, Color.kGreenYellow);
	private LEDAnimation lowRedBlock = SectionManager.LOWBAND.buffer.colorBlockAnimation(0.2, new int[]{5, 5},
			Color.kRed, Color.kMediumVioletRed);

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

			// determine the color of the bands based off the supplier
			// Color sidebandColor = this.sidebandCondition.getAsBoolean() ? Color.kRed :
			// Color.kGreen;

			if (this.sidebandCondition.getAsBoolean()) {
				this.highRedBlock.step();
				this.lowRedBlock.step();
			} else {
				this.highGreenBlock.step();
				this.lowGreenBlock.step();
			}

			// // set the color of the bands
			// // bottom
			// LEDManager.LOWBAND.buffer.setSolidColor(sidebandColor);
			// // top
			// LEDManager.HIGHBAND.buffer.setSolidColor(sidebandColor);
		}

		// this.strip.sendData();
		this.ledSubsystem.sendSectionData();
	}
}
