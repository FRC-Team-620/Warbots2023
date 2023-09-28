package org.jmhsrobotics.frc2023.commands.gripper;

import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.Constants.GripperType;
import org.jmhsrobotics.frc2023.Constants.ScoringType;
import org.jmhsrobotics.frc2023.oi.ControlBoard;
import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopIntakeOpenLoop extends CommandBase {

	IntakeSubsystem intakeSubsystem;
	Supplier<Double> intakeSpeed;
	Supplier<ScoringType> scoringType;
	GripperType type;
	ControlBoard control;

	public TeleopIntakeOpenLoop(IntakeSubsystem intakeSubsystem, GripperType type, Supplier<Double> intakeSpeed,
			Supplier<ScoringType> scoringType) {

		this.intakeSubsystem = intakeSubsystem;
		this.intakeSpeed = intakeSpeed;
		this.scoringType = scoringType;
		this.type = type;

		this.addRequirements(intakeSubsystem);
	}
	public TeleopIntakeOpenLoop(IntakeSubsystem intakeSubsystem, Supplier<Double> intakeSpeed, ControlBoard control) {
		this.intakeSubsystem = intakeSubsystem;
		this.intakeSpeed = intakeSpeed;
		this.control = control;
		this.addRequirements(this.intakeSubsystem);
	}

	@Override
	public void initialize() {
		SmartDashboard.putNumber("TeleopIntake/Max speed", 0.5);
		SmartDashboard.putNumber("TeleopIntake/Min speed", -0.5);
		// SmartDashboard.putString("TeleopIntake/gripper type", this.type.toString());
	}

	@Override
	public void execute() {

		double currentSpeed = this.intakeSpeed.get();
		// ScoringType currentMode = this.scoringType.get();
		// boolean isCone = currentMode == ScoringType.CONE;
		intakeSubsystem.setIntakeMotor(currentSpeed);
		// boolean state = true;
		// if (control.closeGrabber().getAsBoolean()) {
		// state = !state;
		// }
		// state = control.closeGrabber().getAsBoolean() ? !state : state;
		// control.switchGrabber().onTrue(intak.)
		// intakeSubsystem.switchIntakePistonState(control.switchGrabber().getAsBoolean());
		SmartDashboard.putNumber("TeleopIntake/input speed", currentSpeed);
		// SmartDashboard.putString("TeleopIntake/scoring type",
		// currentMode.toString());

		// double baseSpeed;

		// switch (this.type) {

		// case SOLENOID :

		// baseSpeed = 0.05;

		// // if (currentMode == ScoringType.CONE && currentSpeed > 0.0)
		// // currentSpeed *= 0.4;

		// if (!this.intakeSubsystem.getIntakePistonState())
		// baseSpeed = 0.0;

		// // spotless:off
		// 		this.intakeSubsystem.setIntakeMotor(MathUtil.clamp(
		// 			currentSpeed + baseSpeed, -1, 1
		// 		));
		// 		// spotless:on

		// break;

		// case MOTOR :

		// // int modeFactor = currentMode == ScoringType.CONE ? -1 : 1;

		// // baseSpeed = isCone ? 0.05 : 0;

		// // if (isCone)
		// // currentSpeed *= 0.6;

		// // // spotless:off
		// 		// this.intakeSubsystem.setIntakeMotor(
		// 		// 	isCone ? Direction.IN : Direction.OUT, 
		// 		// 	MathUtil.clamp(currentSpeed + baseSpeed, -1, 1));
		// 		// // spotless:on

		// break;
		// }
	}

	@Override
	public void end(boolean interrupted) {
	}
}
