// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.jmhsrobotics.frc2023.Constants.AutoConstants;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

public class AutoDriveDistance extends CommandBase {
	Drivetrain drivetrain;

	private Pose2d initPose;

	private ProfiledPIDController distancePID;

	private double distance;

	int i = 0;

	/** Creates a new AutoDriveDistance. */
	public AutoDriveDistance(Drivetrain drivetrain, double distance) {
		this.drivetrain = drivetrain;
		this.distancePID = new ProfiledPIDController(AutoConstants.autoDistanceKP, AutoConstants.autoDistanceKI,
				AutoConstants.autoDistanceKD,
				new Constraints(AutoConstants.maxVelocity, AutoConstants.maxAcceleration));
		this.distance = distance;
		// distancePID.setGoal(distance);
		distancePID.setTolerance(Units.inchesToMeters(5), AutoConstants.maxVelocity);
		addRequirements(drivetrain);

	}

	@Override
	public void initialize() {
		this.initPose = this.drivetrain.getPose();
		distancePID.setGoal(this.distance);
		// SmartDashboard.putData("DriveDist pid", distancePID); // Add pid loop to
		// glass/SD and network tables
		distancePID.reset(new State(0, 0));
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		var moved = getRelativeDistance();
		double value = this.distancePID.calculate(moved);
		this.drivetrain.setCurvatureDrive(value, 0, false);
		SmartDashboard.putNumber("DriveDist pid/distance", moved);
		SmartDashboard.putNumber("DriveDist pid/goal", distancePID.getGoal().position);
		SmartDashboard.putNumber("DriveDist pid/setpoint2", distancePID.getSetpoint().velocity);
		SmartDashboard.putNumber("DriveDist pid/error", distancePID.getPositionError());
		SmartDashboard.putNumber("DriveDist pid/vel_error", distancePID.getVelocityError());
		SmartDashboard.putNumber("AutoTimer/time", i);
		// System.out.println(this.distancePID.atGoal());
		// i++;
	}

	private double getRelativeDistance() {
		return new Transform2d(initPose, this.drivetrain.getPose()).getTranslation().getX();
	}

	private boolean hasError() {
		return (Math.abs(distancePID.getPositionError()) > distancePID.getPositionTolerance())
				|| (Math.abs(distancePID.getVelocityError()) > distancePID.getVelocityTolerance());
	}

	private boolean atGoalSetpoint() {
		return distancePID.getGoal().equals(distancePID.getSetpoint()) && !hasError();
	}

	// // Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// System.out.println("POS: " + (Math.abs(distancePID.getPositionError()) <
		// distancePID.getPositionTolerance()));
		// System.out.println("VEL: " + (Math.abs(distancePID.getVelocityError()) <
		// distancePID.getVelocityTolerance()));
		// System.out.println("GOL: " + distancePID.atSetpoint());
		// System.out.println("STA: " +
		// (distancePID.getGoal().equals(distancePID.getSetpoint())));
		// // return this.distancePID.atGoal() || i > 200;

		// System.out.println("HAS ERROR: " + hasError());
		// System.out.println("FINAL GOAL: " + atGoalSetpoint());
		return atGoalSetpoint();
	}

	@Override
	public void end(boolean interrupt) {
		// this.distancePID.reset(new State(0, 0));
		// this.distancePID.setGoal(0);
		// i = 0;
		drivetrain.setCurvatureDrive(0, 0, false);
		drivetrain.setBrake(true);
		// System.out.println("DONEDONEDONEDONEDONE\n");
	}
}
