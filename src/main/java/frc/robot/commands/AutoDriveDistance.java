// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveDistance extends CommandBase {
  Drivetrain drivetrain;

  private Pose2d initPose;

  private ProfiledPIDController distancePID;

  /** Creates a new AutoDriveDistance. */
  public AutoDriveDistance(Drivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distancePID = new ProfiledPIDController(AutoConstants.autoDistanceKP, AutoConstants.autoDistanceKI,
        AutoConstants.autoDistanceKD, new Constraints(AutoConstants.maxVelocity, AutoConstants.maxAcceleration));
    distancePID.setGoal(distance);
    distancePID.setTolerance(Units.inchesToMeters(1),0.2);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.initPose = this.drivetrain.getPose();
    SmartDashboard.putData("DriveDist pid", distancePID); // Add pid loop to glass/SD and network tables
    distancePID.reset(new State(0,0));
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var moved = getRelativeDistance();
    double value = this.distancePID.calculate(moved);
    this.drivetrain.setCurvatureDrive(value, 0, false);
    SmartDashboard.putNumber("DriveDist pid/distance", moved);
    SmartDashboard.putNumber("DriveDist pid/goal", distancePID.getGoal().position);
    SmartDashboard.putNumber("DriveDist pid/setpoint2", distancePID.getSetpoint().position);
  }

  private double getRelativeDistance() {
    return new Transform2d(initPose, this.drivetrain.getPose()).getTranslation().getX();
  }

  // // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.distancePID.atGoal();
  }

  @Override
  public void end(boolean interrupt) {
    drivetrain.setCurvatureDrive(0, 0, false);
    drivetrain.setBrake(true);
  }
}