// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIdsMainBot;
import frc.robot.Constants.CANIdsTestBot;

public class Drivetrain extends SubsystemBase {

  // I'm giving you a basic structure and you guys job is to fill it out
  // Hopefully you will learn something from this process and that it will help you with being able to do it on your own
  // Just giving you a helping hand for your first time round

  // Device id's are CAN pin numbers and you will be seeing a lot more of them in the future so I suggest you get used to it 
  // Second argument is a Enum and the long and short of it is it's words that represent a number in a way that makes it more readable, but in this case it's just idenifing that the motor we have plugged into that CAN slot is a brushless motor
  private CANSparkMax leftFrontMotor = new CANSparkMax(CANIdsMainBot.leftFrontMotorCANId, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(CANIdsMainBot.rightFrontMotorCANId, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(CANIdsMainBot.leftRearMotorCANId, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(CANIdsMainBot.rightRearMotorCANId, MotorType.kBrushless);

  private RelativeEncoder leftFrontEncoder;
  private RelativeEncoder rightFrontEncoder;
  private RelativeEncoder leftRearEncoder;
  private RelativeEncoder rightRearEncoder;

  PIDController headingPID;

  AHRS navx;

  private double speedSetpoint = 0.0;
  private double curvatureSetpoint = 0.0;
  private boolean shouldQuickturn = false;

  private double setAngle;
  private boolean isTurning = false;

  DifferentialDrive differentialDrive;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    setupMotors();
    setupFollowerMotors();

    leftFrontEncoder = leftFrontMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    leftRearEncoder = leftRearMotor.getEncoder();
    rightRearEncoder = rightRearMotor.getEncoder();

    //TODO: Use this to maintain gyro heading when going straight or stopped
    headingPID = new PIDController(
      Constants.DriveConstants.kPKeepHeading,
      Constants.DriveConstants.kIKeepHeading,
      Constants.DriveConstants.kDKeepHeading
    );

    navx = new AHRS(Port.kMXP);
    setAngle = navx.getYaw();
    headingPID.setSetpoint(setAngle);

    //Setup differential drive with left front and right front motors as the parameters for the new DifferentialDrive
    differentialDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);
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
    motor.setOpenLoopRampRate(0.2);
    // Smart current limit (limits on the current motors can draw even under full load) = 60
    motor.setSmartCurrentLimit(60);
    // Idle mode (the mode that the motors are in when they are not being told to move) = IdleMode.kBrake
    motor.setIdleMode(IdleMode.kBrake);

    return motor;
  }

  private void setupFollowerMotors() {
    // You need to make the rear motors on both sides of the drivetrain follow their respective front motors
    // This is where we will invert motors as needed when we get to testing the drivetrain
    rightRearMotor.follow(rightFrontMotor);
    leftRearMotor.follow(leftFrontMotor);

    
    rightFrontMotor.setInverted(true);
    leftFrontMotor.setInverted(true);
  }

  public boolean approximatelyZero(double value) {
    return value > -0.01 && value < 0.01;
  }

  @Override
  public void periodic() {

    boolean noCurvatureInput = approximatelyZero(curvatureSetpoint);

    if(this.isTurning && noCurvatureInput) {
      this.setAngle = this.navx.getYaw();
      this.headingPID.setSetpoint(this.setAngle);
      this.isTurning = false;
      System.out.println("ANGLE: " + this.navx.getYaw());
    }

    if(!noCurvatureInput) {
      this.isTurning = true;
    }

    double rotationInput = this.curvatureSetpoint;

    if(!this.isTurning) {
      rotationInput = this.headingPID.calculate(this.navx.getYaw());
    }

    setCurvatureDrive(
      this.speedSetpoint, 
      rotationInput, 
      this.shouldQuickturn
    );
  }

  public void stop() {
    this.speedSetpoint = 0.0;
    this.curvatureSetpoint = 0.0;
    this.shouldQuickturn = false;
  }

  public void setSpeed(double speed) {
    this.speedSetpoint = speed;
  }

  public void setCurvature(double curvature) {
    this.curvatureSetpoint = curvature;
  }

  public void setQuickturn(boolean quickturn) {
    this.shouldQuickturn = quickturn;
  }

  public void resetEncoders() {
    leftFrontEncoder.setPosition(0.0);
    rightFrontEncoder.setPosition(0.0);
    leftRearEncoder.setPosition(0.0);
    rightRearEncoder.setPosition(0.0);
  }
 
  public double getRightEncoderCount() {
    return (rightFrontEncoder.getPosition() + 
      rightRearEncoder.getPosition()) / 2.0;
  }

  public double getLeftEncoderCount() {
    return (leftFrontEncoder.getPosition() + 
      leftRearEncoder.getPosition()) / 2.0;
  }

  //Sets the differential drive using the method curvatureDrive
  public void setCurvatureDrive(double speed, double rotationInput, boolean quickTurn) {
    differentialDrive.curvatureDrive(speed, rotationInput, quickTurn);
  }
}
