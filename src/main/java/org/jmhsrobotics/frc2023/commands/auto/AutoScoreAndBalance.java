package org.jmhsrobotics.frc2023.commands.auto;

import org.jmhsrobotics.frc2023.NewConstants;
import org.jmhsrobotics.frc2023.commands.arm.CommandArmPitchSimple;
import org.jmhsrobotics.frc2023.commands.wrist.CommandSetGripperOpen;
import org.jmhsrobotics.frc2023.commands.wrist.CommandWristSimple;
import org.jmhsrobotics.frc2023.commands.wrist.CommandWristSimpleDefualt;
import org.jmhsrobotics.frc2023.subsystems.ArmPitchSubsystem;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreAndBalance extends SequentialCommandGroup {
	private Drivetrain drivetrain;
	private WristSubsystem wrist;
	private IntakeSubsystem intake;
	private ArmPitchSubsystem armPitch;

	public AutoScoreAndBalance(Drivetrain drivetrain, WristSubsystem wrist, IntakeSubsystem intake,
			ArmPitchSubsystem armPitch) {
		this.drivetrain = drivetrain;
		this.wrist = wrist;
		this.intake = intake;
		this.armPitch = armPitch;

		// addCommands(new CenterChargeStationAuto(this.drivetrain));
		addCommands(setArmWristScoringPosition(), setArmWristStowedPosition(),
				new CenterChargeStationAuto(this.drivetrain));

	}

	private SequentialCommandGroup setArmWristScoringPosition() {
		return new SequentialCommandGroup(new CommandArmPitchSimple(this.armPitch, NewConstants.ARM_PRESET_MID),
				new CommandWristSimple(this.wrist, NewConstants.WRIST_PRESET_MID),
				new CommandSetGripperOpen(this.intake, true));
	}

	private SequentialCommandGroup setArmWristStowedPosition() {
		return new SequentialCommandGroup(new CommandWristSimple(this.wrist, NewConstants.WRIST_PRESET_STOWED),
				new CommandArmPitchSimple(this.armPitch, NewConstants.ARM_PRESET_STOWED),
				new CommandSetGripperOpen(this.intake, false), new CommandWristSimpleDefualt(this.wrist));
	}
}
