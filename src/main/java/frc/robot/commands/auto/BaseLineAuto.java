package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDriveDistance;
import frc.robot.subsystems.Drivetrain;

public class BaseLineAuto extends SequentialCommandGroup {

  Drivetrain drivetrain;

  //Constructor 
  public BaseLineAuto(Drivetrain drivetrain){
    this.drivetrain = drivetrain;
    addCommands(
        new InstantCommand(()-> {
            drivetrain.resetOdometry(new Pose2d(Units.inchesToMeters( 54.42), Units.inchesToMeters( 42.079), Rotation2d.fromDegrees(180)));
        }),
        new AutoDriveDistance(drivetrain, -4.5),
        new WaitCommand(0.5),
        new AutoDriveDistance(drivetrain, 3.5)
    );
  }

}


