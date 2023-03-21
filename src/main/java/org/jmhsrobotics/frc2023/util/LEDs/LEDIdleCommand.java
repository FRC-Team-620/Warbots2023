package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDManager;

// import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDManager;

public class LEDIdleCommand extends CommandBase {

	// LEDSection strip = LEDManager.STRIP0.strip;
	LEDBuffer midBuffer = LEDManager.BODY.buffer;
	BooleanSupplier condition, sidebandCondition;

	private final int sidebandLength = 10;

	private LEDAnimation fadeAnim = midBuffer.fadeAnimation(1, 30, Color.kYellow, Color.kWhite);
	private LEDAnimation redGradAnim = midBuffer.gradientAnimation(1, Color.kRed, Color.kOrangeRed, Color.kYellow);
	private LEDAnimation blueGradAnim = midBuffer.gradientAnimation(1, Color.kBlue, Color.kBlueViolet, Color.kPurple);
	private LEDAnimation solidAnim = midBuffer.solidColorAnimation(Color.kBlue);

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
				this.redGradAnim.step(); // RED if true
			} else {
				this.blueGradAnim.step(); // BLUE if false
			}
		}

		if (this.sidebandCondition != null) {

			// determine the color of the bands based off the supplier
			Color sidebandColor = this.sidebandCondition.getAsBoolean() ? Color.kRed : Color.kGreen;

			// set the color of the bands
			// bottom
			LEDManager.LOWBAND.buffer.setSolidColor(sidebandColor);
			// top
			LEDManager.HIGHBAND.buffer.setSolidColor(sidebandColor);
		}

		// this.strip.sendData();
		this.ledSubsystem.sendData();
	}
}
