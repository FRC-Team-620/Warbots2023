package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDManager;

// import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDManager;

public class LEDIdleCommand extends CommandBase {

	// LEDSection strip = LEDManager.STRIP0.strip;
	LEDBuffer strip0 = LEDManager.STRIP0.buffer;
	BooleanSupplier condition, sidebandCondition;

	private final int sidebandLength = 10;

	private LEDAnimation fadeAnim = this.strip0.fadeAnimation(1, 30, Color.kYellow, Color.kWhite);
	private LEDAnimation redGradAnim = this.strip0.gradientAnimation(1, Color.kRed, Color.kOrangeRed, Color.kYellow);
	private LEDAnimation blueGradAnim = this.strip0.gradientAnimation(1, Color.kBlue, Color.kBlueViolet, Color.kPurple);
	private LEDAnimation solidAnim = this.strip0.solidColorAnimation(Color.kBlue);

	LEDSubsystem ledSubsystem;

	public LEDIdleCommand(LEDSubsystem ledSubsystem, BooleanSupplier condition, BooleanSupplier sidebandCondition) {

		this.ledSubsystem = ledSubsystem;

		this.condition = condition;
		this.sidebandCondition = sidebandCondition;

		addRequirements(this.ledSubsystem);
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

		if (this.sidebandCondition != null && this.strip0.getLength() > this.sidebandLength) {

			// determine the color of the bands based off the supplier
			Color sidebandColor = this.sidebandCondition.getAsBoolean() ? Color.kRed : Color.kGreen;

			// set the color of the bands
			// bottom
			this.strip0.setSubsetSolidColor(0, this.sidebandLength, sidebandColor);
			// top
			this.strip0.setSubsetSolidColor(this.strip0.getLength() - this.sidebandLength, this.strip0.getLength(),
					sidebandColor);
		}

		// this.strip.sendData();
		this.ledSubsystem.sendData();
	}
}
