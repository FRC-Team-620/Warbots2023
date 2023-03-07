package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.jmhsrobotics.frc2023.RobotContainer;
import org.jmhsrobotics.frc2023.commands.AutoDriveDistance;
import org.jmhsrobotics.frc2023.commands.auto.ScoringAuto.startingPositions;

public class AutoSelector {
	SendableChooser<CommandBase> autoSelector = new SendableChooser<>();
	SendableChooser<DriverStation.Alliance> teamSelector = new SendableChooser<>();
	SendableChooser<Preload> preloadedSelector = new SendableChooser<>();

	private enum Preload {
		CONE, CUBE, NOTHING
	}

	public AutoSelector(RobotContainer container) {
		// Add auto Options
		autoSelector.setDefaultOption("StartingDistanceAuto", new StartingDistanceAuto(container.getDrivetrain()));
		autoSelector.addOption("CenterChargeStationAuto", new CenterChargeStationAuto(container.getDrivetrain(), container.getLEDSubsystem()));
		autoSelector.addOption("ScoringAutoLeft",
				new ScoringAuto(container.drivetrain, container.armSubsystem, container.ledSubsystem, startingPositions.LEFT));
		autoSelector.addOption("ScoringAutoCenter",
				new ScoringAuto(container.drivetrain, container.armSubsystem, container.ledSubsystem, startingPositions.CENTER));
		autoSelector.addOption("ScoringAutoLeft",
				new ScoringAuto(container.drivetrain, container.armSubsystem, container.ledSubsystem, startingPositions.RIGHT));		
		SmartDashboard.putData("autoSelector", autoSelector);

		// Add Selector for Alliance color TODO: Use DriveStation.getAlliance() to set
		// default
		teamSelector.setDefaultOption("Blue Alliance", DriverStation.Alliance.Blue);
		teamSelector.addOption("Red Alliance", DriverStation.Alliance.Red);
		SmartDashboard.putData("teamSelector", teamSelector);

		// Add Selector for Starting game piece
		preloadedSelector.addOption("Cone", Preload.CONE);
		preloadedSelector.addOption("Cube", Preload.CUBE);
		preloadedSelector.addOption("Nothing", Preload.NOTHING);
		SmartDashboard.putData("preloadedSelector", preloadedSelector);
	}

	public CommandBase getCommand() {
		return autoSelector.getSelected();
	}
}
