package org.jmhsrobotics.frc2023.commands.auto;

import java.nio.file.Path;

import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoDriveTrajectory extends SequentialCommandGroup {

	public AutoDriveTrajectory(Drivetrain drivetrain, String path, boolean zeroGyro) {
		Trajectory trajectory = new Trajectory();

		try {
			System.out.println("Loading trajectory...");
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path + ".wpilib.json");
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			System.out.println("Done loading trajectory!");
			Pose2d initialPose = trajectory.getInitialPose();

			SequentialCommandGroup pathCommand = drivetrain.createRamseteCommand(trajectory);

			addCommands(new InstantCommand(() -> {
				drivetrain.resetOdometry(initialPose);
			}, drivetrain), pathCommand
			// new InstantCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain)
			);

		} catch (Exception ex) {
			DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
		}

	}

}
