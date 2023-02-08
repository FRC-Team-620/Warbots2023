package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.jmhsrobotics.frc2023.RobotContainer;
import org.jmhsrobotics.frc2023.commands.AutoDriveDistance;

public class AutoSelector {
	SendableChooser<CommandBase> autoSelector = new SendableChooser<>();
	SendableChooser<DriverStation.Alliance> teamSelector = new SendableChooser<>();
	SendableChooser<Preload> preloadedSelector = new SendableChooser<>();

	private enum Preload {
		CONE, CUBE, NOTHING
	}

	public AutoSelector(RobotContainer robotContainer) {
		// Add auto Options
		autoSelector.setDefaultOption("BaseLineAuto", new BaseLineAuto(robotContainer.getDrivetrain()));
		autoSelector.addOption("AutoDriveDistance 2 m", new AutoDriveDistance(robotContainer.getDrivetrain(), 2));
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
