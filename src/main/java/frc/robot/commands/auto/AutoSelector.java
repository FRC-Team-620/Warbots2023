package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveDistance;

public class AutoSelector {
    SendableChooser<CommandBase> autoSelector = new SendableChooser<>();
    SendableChooser<DriverStation.Alliance> teamSelector = new SendableChooser<>();
    SendableChooser<Preload> preloadedSelector = new SendableChooser<>();

    private enum Preload {
        CONE, CUBE, NOTHING
    }

    public AutoSelector(RobotContainer container) {
        autoSelector.setDefaultOption("BaseLineAuto", new BaseLineAuto(container.getDrivetrain()));
        autoSelector.addOption("AutoDriveDistance 2 m", new AutoDriveDistance(container.getDrivetrain(), 2));
        SmartDashboard.putData("autoSelector", autoSelector);
        // teamSelector.setDefaultOption("", DriverStation.getAlliance());
        teamSelector.setDefaultOption("Blue Alliance", DriverStation.Alliance.Blue);
        teamSelector.addOption("Red Alliance", DriverStation.Alliance.Red);
        SmartDashboard.putData("teamSelector",teamSelector);

        preloadedSelector.addOption("Cone", Preload.CONE);
        preloadedSelector.addOption("Cube", Preload.CUBE);        
        preloadedSelector.addOption("Nothing", Preload.NOTHING);
        SmartDashboard.putData("preloadedSelector", preloadedSelector);
    }

    public CommandBase getCommand() {
        return autoSelector.getSelected();
    }
}
