// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveDistance extends CommandBase {
  Drivetrain drivetrain;
  private double distance;
  private double autoSpeed = 0.5;

  private Pose2d initPose;
  
  private PIDController distancePID;
  private TrapezoidProfile profile;
  private Timer timer;
  // protected final PIDController leftPID;
  // protected final PIDController rightPID;
  /** Creates a new AutoDriveDistance. */
  public AutoDriveDistance(Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.distancePID = new PIDController(AutoConstants.autoDistanceKP, AutoConstants.autoDistanceKI, AutoConstants.autoDistanceKD);
    this.profile = new TrapezoidProfile(new Constraints(AutoConstants.maxVelocity, AutoConstants.maxAcceleration), new State(distance, 0));
    this.timer = new Timer();
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.initPose = this.drivetrain.getPose();
    timer.start();
  }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double setpoint = this.profile.calculate(timer.get()).position;
    System.out.println(timer.get());
    double moved = this.initPose.getTranslation().getDistance(this.drivetrain.getPose().getTranslation());
    double value = this.distancePID.calculate(moved, setpoint);
    System.out.println(value + " " + moved + " " + setpoint);
    this.drivetrain.setCurvatureDrive(value, 0, false);
  }

  public boolean withinBounds() {
    if (this.getDisplacement() > distance) {
      return true;
    }
    return false;
  }

  private double getDisplacement() {
    return this.initPose.getTranslation().getDistance(this.drivetrain.getPose().getTranslation());
  }




  // public double averageDistance() {
  //   return (drivetrain.getLeftPosition() + drivetrain.getRightPosition()) / 2;
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return withinBounds();
  }

  @Override
  public void end(boolean interrupt) {
    drivetrain.setCurvatureDrive(0, 0, false);
  }
}