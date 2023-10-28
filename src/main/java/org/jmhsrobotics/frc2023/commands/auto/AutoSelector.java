package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.jmhsrobotics.frc2023.RobotContainer;
//import org.jmhsrobotics.frc2023.commands.auto.ScoringAuto.StartingPosition;

public class AutoSelector {
	SendableChooser<CommandBase> autoSelector = new SendableChooser<>();
	SendableChooser<DriverStation.Alliance> teamSelector = new SendableChooser<>();
	SendableChooser<Preload> preloadedSelector = new SendableChooser<>();

	private enum Preload {
		CONE, CUBE, NOTHING
	}

	public AutoSelector(RobotContainer container) {
		// Add auto Options
		autoSelector.setDefaultOption("CenterChargeStationAuto",
				new CenterChargeStationAuto(container.getDrivetrain()));

		autoSelector.addOption("StartingDistanceAuto", new StartingDistanceAuto(container.getDrivetrain()));
		autoSelector.addOption("DoNothing", new InstantCommand());
		// scoring + auto balance auto
		autoSelector.addOption("score+balance", new AutoScoreAndBalance(container.getDrivetrain(),
				container.getWristSubsystem(), container.getIntakeSubsystem(), container.getArmPitchSubysystem()));
		autoSelector.addOption("score+drive", new AutoScoreAndDrive(container.getDrivetrain(),
				container.getWristSubsystem(), container.getIntakeSubsystem(), container.getArmPitchSubysystem()));
		SmartDashboard.putData("autoSelector", autoSelector);
	}

	public CommandBase getCommand() {
		return autoSelector.getSelected();
	}
}
