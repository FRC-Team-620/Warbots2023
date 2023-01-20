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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DiminishingAverageHandler;
import frc.robot.Constants.CANIdsMainBot;
import frc.robot.Constants.CANIdsTestBot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotMath;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

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

  private PIDController headingPID; // SETPOINT IS ALWAYS 0 (we give it a relative angle)

  private AHRS navx;

  private double speedSetpoint = 0.0;
  private double curvatureSetpoint = 0.0;
  private boolean shouldQuickturn = false;

  private double angularVelocity = 0.0;

  private double setAngle;
  private boolean isTurning = false;
  private boolean isStopping = false;
  // private int tickAccumulation = 0;
  private double previousAngle;

  private DiminishingAverageHandler angularVelocityHandler;
  private DifferentialDrive differentialDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    setupMotors();
    setupFollowerMotors();

    leftFrontEncoder = leftFrontMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    leftRearEncoder = leftRearMotor.getEncoder();
    rightRearEncoder = rightRearMotor.getEncoder();

    headingPID = new PIDController(
      Constants.DriveConstants.kPKeepHeading,
      Constants.DriveConstants.kIKeepHeading,
      Constants.DriveConstants.kDKeepHeading
    );
    headingPID.setSetpoint(0.0); // IMPORTANT

    navx = new AHRS(Port.kMXP);
    setAngle = this.getYaw();
    SmartDashboard.putNumber("heading_angle", 0.0);

    this.angularVelocityHandler = new DiminishingAverageHandler(2);

    //Setup differential drive with left front and right front motors as the parameters for the new DifferentialDrive
    differentialDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);
  }

  private void setupMotors() {
    leftFrontMotor = setupMotor(leftFrontMotor);
    rightFrontMotor = setupMotor(rightFrontMotor);
    leftRearMotor = setupMotor(leftRearMotor);
    rightRearMotor = setupMotor(rightRearMotor);
  }
  public void setBrake(boolean brake){
    IdleMode mode = IdleMode.kCoast;
    if (brake){
      mode = IdleMode.kBrake;
    }
    leftFrontMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    leftRearMotor.setIdleMode(mode);
    rightRearMotor.setIdleMode(mode);
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

  @Override
  public void periodic() {

    double yaw = this.getYaw();

    double relativeChange = RobotMath.relativeAngle(this.previousAngle, yaw);

    this.angularVelocity = this.angularVelocityHandler.feed(
      relativeChange / RobotConstants.secondsPerTick
    );
    this.previousAngle = yaw;

    SmartDashboard.putNumber("angular_velocity", this.angularVelocity);

    boolean noCurvatureInput = RobotMath.approximatelyZero(curvatureSetpoint);

    if(this.isStopping && noCurvatureInput && 
      RobotMath.approximatelyZero(this.angularVelocity, 0.2)) {

      this.setAngle = yaw;
      this.isStopping = false;
      // this.tickAccumulation = 0;
      System.out.println("SET STOPPED PIVOT ANGLE:  " + yaw);
    }

    if(!noCurvatureInput) { // YES curvature input
      this.isTurning = true;
    } else if(this.isTurning) { // no curvature input, isTurning is true
      isStopping = true;
      this.isTurning = false;
      this.setAngle = yaw;
      System.out.println("SET SLOWDOWN PIVOT ANGLE:  " + yaw);
      // this.tickAccumulation++;
    }

    double rotationInput = this.curvatureSetpoint;

    if(!this.isTurning) { // NOT TURNING
      double relativeAngle = RobotMath.relativeAngle(this.setAngle, yaw);
      rotationInput = this.headingPID.calculate(relativeAngle);
      SmartDashboard.putNumber("set_angle", setAngle);
      SmartDashboard.putNumber("heading_angle", yaw);
      SmartDashboard.putNumber("relative_angle", relativeAngle);
    }
    
    setCurvatureDrive(
      this.speedSetpoint, 
      rotationInput, 
      this.shouldQuickturn
    );
  }

  /**
   * Getter for the angular velocity of the robot in degrees per second
   * 
   * @return Angular velocity for the yaw in degrees per second
   */
  public double getAngularVelocity() {
    return this.angularVelocity;
  }

  public void resetAngularVelocity() {
    this.angularVelocityHandler.reset();
  }

  public double getYaw() {
    return this.navx.getYaw();
  }

  public void stop() {
    this.speedSetpoint = 0.0;
    this.curvatureSetpoint = 0.0;
    this.shouldQuickturn = false;
  }

  public void setCurrentAngle(double angle) {
    this.setAngle = angle;
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
