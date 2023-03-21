package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDManager;

public class LEDIdleCommand extends CommandBase {

	LEDStrip strip = LEDManager.STRIP0.strip;
	BooleanSupplier condition, sidebandCondition;

	private final int sidebandLength = 10;

	private LEDAnimation fadeAnim = this.strip.fadeAnimation(1, 30, Color.kYellow, Color.kWhite);
	private LEDAnimation redGradAnim = this.strip.gradientAnimation(1, Color.kRed, Color.kOrangeRed, Color.kYellow);
	private LEDAnimation blueGradAnim = this.strip.gradientAnimation(1, Color.kBlue, Color.kBlueViolet, Color.kPurple);
	private LEDAnimation solidAnim = this.strip.solidColorAnimation(Color.kBlue);

	public LEDIdleCommand(LEDSubsystem ledSubsystem, BooleanSupplier condition, BooleanSupplier sidebandCondition) {

		this.condition = condition;
		this.sidebandCondition = sidebandCondition;

		addRequirements(ledSubsystem);
	}

	@Override
	public void execute() {

		if (this.condition != null) {
			if (this.condition.getAsBoolean()) {
				this.redGradAnim.step(); // RED if true
			} else {
				this.blueGradAnim.step(); // BLUE if false
			}
		}

		if (this.sidebandCondition != null && this.strip.getLength() > this.sidebandLength) {

			// determine the color of the bands based off the supplier
			Color sidebandColor = this.sidebandCondition.getAsBoolean() ? Color.kRed : Color.kGreen;

			// set the color of the bands
			// bottom
			this.strip.setSubsetSolidColor(0, this.sidebandLength, sidebandColor);
			// top
			this.strip.setSubsetSolidColor(this.strip.getLength() - this.sidebandLength, this.strip.getLength(),
					sidebandColor);
		}

		this.strip.sendData();
	}
}
