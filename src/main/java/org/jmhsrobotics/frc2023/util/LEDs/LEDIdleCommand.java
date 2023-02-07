package org.jmhsrobotics.frc2023.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDAnimation;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDManager;

public class LEDIdleCommand extends CommandBase {

	LEDSubsystem.LEDStrip strip = LEDManager.STRIP0.strip;
	BooleanSupplier isTurningSupplier;

	private LEDAnimation fadeAnim = this.strip.fadeAnimation(1, 30, Color.kYellow, Color.kWhite);
	private LEDAnimation redGradAnim = this.strip.gradientAnimation(1, Color.kRed, Color.kOrangeRed, Color.kOrange);
	private LEDAnimation blueGradAnim = this.strip.gradientAnimation(1, Color.kBlue, Color.kBlueViolet, Color.kPurple);
	private LEDAnimation solidAnim = this.strip.solidColorAnimation(Color.kBlue);

	public LEDIdleCommand(LEDSubsystem ledSubsystem, Drivetrain drivetrain) {

		this.isTurningSupplier = () -> drivetrain.getIsTurning();

		addRequirements(ledSubsystem);
	}

	@Override
	public void execute() {

		if (this.isTurningSupplier.getAsBoolean()) {
			this.redGradAnim.step();
		} else {
			this.blueGradAnim.step();
		}
	}
}
