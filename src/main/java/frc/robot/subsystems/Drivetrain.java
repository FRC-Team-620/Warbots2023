// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdsTestBot;

public class Drivetrain extends SubsystemBase {

  // I'm giving you a basic structure and you guys job is to fill it out
  // Hopefully you will learn something from this process and that it will help you with being able to do it on your own
  // Just giving you a helping hand for your first time round

  // Device id's are CAN pin numbers and you will be seeing a lot more of them in the future so I suggest you get used to it 
  // Second argument is a Enum and the long and short of it is it's words that represent a number in a way that makes it more readable, but in this case it's just idenifing that the motor we have plugged into that CAN slot is a brushless motor
  private CANSparkMax leftFrontMotor = new CANSparkMax(CANIdsTestBot.leftFrontMotorCANId, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(CANIdsTestBot.rightFrontMotorCANId, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(CANIdsTestBot.leftRearMotorCANId, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(CANIdsTestBot.rightRearMotorCANId, MotorType.kBrushless);

  DifferentialDrive differentialDrive;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    setupMotors();
    setupFollowerMotors();
    //Setup differential drive with left front and right front motors as the parameters for the new DifferentialDrive
  }
  private void setupMotors() {
    leftFrontMotor = setupMotor(leftFrontMotor);
    rightFrontMotor = setupMotor(rightFrontMotor);
    leftRearMotor = setupMotor(leftRearMotor);
    rightRearMotor = setupMotor(rightRearMotor);
  }
  private CANSparkMax setupMotor(CANSparkMax motor) {
    // You need to make the motor have the following settings that you can set through the various motor methods: 
    // Open loop ramp rate (time it takes to reach max acceleration in seconds) = 0.2 
    // Smart current limit (limits on the current motors can draw even under full load) = 60
    // Idle mode (the mode that the motors are in when they are not being told to move) = IdleMode.kBrake
    return motor;
  }

  private void setupFollowerMotors() {
    // You need to make the rear motors on both sides of the drivetrain follow their respective front motors
    // This is where we will invert motors as needed when we get to testing the drivetrain
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Sets the differential drive using the method curvatureDrive
  public void setCurvatureDrive(double speed, double rotationInput, boolean quickTurn) {
    
  }
}
