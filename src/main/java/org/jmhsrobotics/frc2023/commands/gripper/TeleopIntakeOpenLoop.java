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
		intakeSubsystem.setIntakeMotor(currentSpeed);
		SmartDashboard.putNumber("TeleopIntake/input speed", currentSpeed);
	}

	@Override
	public void end(boolean interrupted) {
	}
}
