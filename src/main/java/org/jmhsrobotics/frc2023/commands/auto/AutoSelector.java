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
		// autoSelector.addOption("ScoringAutoMidCone",
		// new ScoringAuto(container.getDrivetrain(), container.getWristSubsystem(),
		// container.getArmSubsystem(),
		// container.getIntakeSubsystem(), container.getLEDSubsystem(),
		// StartingPosition.LEFT,
		// container.getControls(), container, ScoringType.CONE, Setpoints.MID));
		// autoSelector.addOption("ScoringAutoHighCubes",
		// new ScoringAuto(container.getDrivetrain(), container.getWristSubsystem(),
		// container.getArmSubsystem(),
		// container.getIntakeSubsystem(), container.getLEDSubsystem(),
		// StartingPosition.CENTER,
		// container.getControls(), container, ScoringType.CUBE, Setpoints.HIGH));
		// autoSelector.addOption("ScoringAutoMidCubes",
		// new ScoringAuto(container.getDrivetrain(), container.getWristSubsystem(),
		// container.getArmSubsystem(),
		// container.getIntakeSubsystem(), container.getLEDSubsystem(),
		// StartingPosition.RIGHT,
		// container.getControls(), container, ScoringType.CUBE, Setpoints.MID));
		SmartDashboard.putData("autoSelector", autoSelector);

		// Add Selector for Alliance color TODO: Use DriveStation.getAlliance() to
		// set
		// default
		// teamSelector.setDefaultOption("Blue Alliance", DriverStation.Alliance.Blue);
		// teamSelector.addOption("Red Alliance", DriverStation.Alliance.Red);
		// SmartDashboard.putData("teamSelector", teamSelector);

		// // Add Selector for Starting game piece
		// preloadedSelector.addOption("Cone", Preload.CONE);
		// preloadedSelector.addOption("Cube", Preload.CUBE);
		// preloadedSelector.addOption("Nothing", Preload.NOTHING);
		// SmartDashboard.putData("preloadedSelector", preloadedSelector);
	}

	public CommandBase getCommand() {
		return autoSelector.getSelected();
	}
}
