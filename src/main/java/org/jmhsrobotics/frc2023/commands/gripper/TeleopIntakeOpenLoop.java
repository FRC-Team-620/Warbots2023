package org.jmhsrobotics.frc2023.commands.gripper;

import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.Constants.GripperType;
import org.jmhsrobotics.frc2023.Constants.ScoringType;
import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopIntakeOpenLoop extends CommandBase {

	IntakeSubsystem intakeSubsystem;
	Supplier<Double> intakeSpeed;
	Supplier<ScoringType> scoringType;
	GripperType type;

	public TeleopIntakeOpenLoop(IntakeSubsystem intakeSubsystem, GripperType type, Supplier<Double> intakeSpeed,
			Supplier<ScoringType> scoringType) {

		this.intakeSubsystem = intakeSubsystem;
		this.intakeSpeed = intakeSpeed;
		this.scoringType = scoringType;
		this.type = type;

		this.addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		SmartDashboard.putString("TeleopIntake/gripper type", this.type.toString());
	}

	@Override
	public void execute() {

		double currentSpeed = this.intakeSpeed.get();
		ScoringType currentMode = this.scoringType.get();

		SmartDashboard.putNumber("TeleopIntake/input speed", currentSpeed);
		SmartDashboard.putString("TeleopIntake/scoring type", currentMode.toString());

		double baseSpeed;

		switch (this.type) {

			case SOLENOID :

				baseSpeed = 0.05;

				if (currentMode == ScoringType.CONE && currentSpeed > 0.0)
					currentSpeed *= 0.4;

				if (!this.intakeSubsystem.getIntakePistonState())
					baseSpeed = 0.0;

				// spotless:off
				this.intakeSubsystem.setIntakeMotor(MathUtil.clamp(
					currentSpeed + baseSpeed, -1, 1
				));
				// spotless:on

				break;

			case MOTOR :

				int modeFactor = currentMode == ScoringType.CONE ? -1 : 1;

				baseSpeed = currentMode == ScoringType.CONE ? modeFactor * 0.1 : 0;

				if (currentMode == ScoringType.CONE)
					currentSpeed *= 0.6;

				// spotless:off
				this.intakeSubsystem.setIntakeMotor(MathUtil.clamp(
					modeFactor * currentSpeed + baseSpeed, -1, 1
				));
				// spotless:on

				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
	}
}
