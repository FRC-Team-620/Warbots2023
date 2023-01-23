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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveDistance extends CommandBase {//Not WORKING DO NOT USE PLEASE
  Drivetrain drivetrain;
  private double distance;
  private double autoSpeed = 0.5;

  private Pose2d initPose;
  
  private ProfiledPIDController distancePID;
  private Timer timer;
  // protected final PIDController leftPID;
  // protected final PIDController rightPID;
  /** Creates a new AutoDriveDistance. */
  public AutoDriveDistance(Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.distancePID = new ProfiledPIDController(AutoConstants.autoDistanceKP, AutoConstants.autoDistanceKI, AutoConstants.autoDistanceKD, new Constraints(AutoConstants.maxVelocity, AutoConstants.maxAcceleration));
    distancePID.setGoal(distance);
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
    // System.out.println(timer.get());
    
    double value = this.distancePID.calculate(getRelativeDistance());
    // System.out.println(value + " " + moved + " " + setpoint);
    this.drivetrain.setCurvatureDrive(value, 0, false);
  }

  private double getRelativeDistance(){
    return new Transform2d(initPose,this.drivetrain.getPose()).getTranslation().getX();
  }
  public boolean withinBounds() {
    return this.distancePID.atGoal();
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
    System.out.println("Displacement: " + this.getDisplacement());
    return this.withinBounds();
  }

  @Override
  public void end(boolean interrupt) {
    drivetrain.setCurvatureDrive(0, 0, false);
    drivetrain.setBrake(true);
  }
}