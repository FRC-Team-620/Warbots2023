// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.WheelConstants;
import frc.robot.RobotMath;
import frc.robot.util.IIMUWrapper;
import frc.robot.util.sim.NavxWrapper;
import frc.robot.util.sim.RevEncoderSimWrapper;

public class Drivetrain extends SubsystemBase {
  
  // I'm giving you a basic structure and you guys job is to fill it out
  // Hopefully you will learn something from this process and that it will help you with being able to do it on your own
  // Just giving you a helping hand for your first time round

  // Device id's are CAN pin numbers and you will be seeing a lot more of them in the future so I suggest you get used to it 
  // Second argument is a Enum and the long and short of it is it's words that represent a number in a way that makes it more readable, but in this case it's just idenifing that the motor we have plugged into that CAN slot is a brushless motor
  private CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveports.getLeftFrontMotorCANId(), MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveports.getRightFrontMotorCANId(), MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(Constants.driveports.getLeftRearMotorCANId(), MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(Constants.driveports.getRightRearMotorCANId(), MotorType.kBrushless);

  private RelativeEncoder leftFrontEncoder;
  private RelativeEncoder leftRearEncoder;
  private RelativeEncoder rightFrontEncoder;
  private RelativeEncoder rightRearEncoder;
  private IIMUWrapper imu = Constants.driveports.getIMU();
  
  private PIDController headingPID;
  private DifferentialDriveOdometry odometry;
  
  // private double commandedXSpeed = 0.0;
  // private double commandedZRotation = 0.0;
  // private boolean commandedAllowTurnInPlace = false;
  
  private double angularVelocity = 0.0;

  // private boolean headingLock = false;

  private boolean isTurning = false;
  // private int tickAccumulation = 0;
  private double previousAngle;

  private double speedSetpoint = 0.0;
  private double curvatureSetpoint = 0.0;
  private boolean shouldQuickturn = false;

  private boolean shouldHeadingLock = true;

  private RobotMath.DiminishingAverageHandler angularVelocityHandler;
  private DifferentialDrive differentialDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    SmartDashboard.putNumber("Drivetrain/leftFrontCANID", leftFrontMotor.getDeviceId());
    SmartDashboard.putNumber("Drivetrain/rightFrontCANID", rightFrontMotor.getDeviceId());
    SmartDashboard.putNumber("Drivetrain/leftRearCANID", leftRearMotor.getDeviceId());
    SmartDashboard.putNumber("Drivetrain/rightRearCANID", rightRearMotor.getDeviceId());
    
    setupMotors();
    setupFollowerMotors();
    initSensors();

    headingPID = new PIDController(
      Constants.driveports.getkPKeepHeading(),
      Constants.driveports.getkIKeepHeading(),
      Constants.driveports.getkDKeepHeading()
    );
    SmartDashboard.putData("DriveTrainHeading", headingPID);
    
    headingPID.enableContinuousInput(-180, 180);
    headingPID.setTolerance(2,1);

    this.lockCurrentHeading();

    this.angularVelocityHandler = new RobotMath.DiminishingAverageHandler(2.0);

    //Setup differential drive with left front and right front motors as the parameters for the new DifferentialDrive
    differentialDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);
  }

  public void setBrake(boolean brake){
    // if (brake){
    //   mode = IdleMode.kBrake;
    // }
    // else{
    //   mode = IdleMode.kCoast;
    // }
    IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;
    leftFrontMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    leftRearMotor.setIdleMode(mode);
    rightRearMotor.setIdleMode(mode);
  }

  public double getPitch(){
    return imu.getPitch();
  }

  private void setupMotors() {
    leftFrontMotor = setupMotor(leftFrontMotor);
    rightFrontMotor = setupMotor(rightFrontMotor);
    leftRearMotor = setupMotor(leftRearMotor);
    rightRearMotor = setupMotor(rightRearMotor);
  }

  private void initSensors() {
    leftFrontEncoder = leftFrontMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    leftRearEncoder = leftRearMotor.getEncoder();
    rightRearEncoder = rightRearMotor.getEncoder();

    SmartDashboard.putNumber("Drivetrain/ConversionFactor", WheelConstants.conversionFactor);

    leftFrontEncoder.setPositionConversionFactor(WheelConstants.conversionFactor);
    rightFrontEncoder.setPositionConversionFactor(WheelConstants.conversionFactor);
    leftRearEncoder.setPositionConversionFactor(WheelConstants.conversionFactor);
    rightRearEncoder.setPositionConversionFactor(WheelConstants.conversionFactor);

    //leftFrontEncoder = leftFrontMotor.getEncoder();
    //rightFrontEncoder = rightFrontMotor.getEncoder();

    //leftFrontEncoder.setPositionConversionFactor(DriveConstants.metersPerEncoderTick);
    //rightFrontEncoder.setPositionConversionFactor(DriveConstants.metersPerEncoderTick);
    
    odometry = new DifferentialDriveOdometry(imu.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
  }
  
  @Override
  public void periodic() {
      // Update the odometry
      odometry.update(imu.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

      double yaw = this.getYaw();
      
      // get the change in angles, used to calculate the momentary angular velocity
      double relativeChange = RobotMath.relativeAngle(this.previousAngle, yaw);

      // get the angular velocity, denoised with a diminishing average loop
      this.angularVelocity = this.angularVelocityHandler.feed(
        relativeChange / RobotConstants.secondsPerTick
      );

      // save the current angle so next loop it can be used in the above loop to calculate angular velocity
      this.previousAngle = yaw;


      // check whether there is curvature input 
      boolean noCurvatureInput = RobotMath.approximatelyZero(this.curvatureSetpoint);

      // If the robot is still registering itself as spinning (so it was spinning), 
      // but there is no input, and there is no angular velocity 
      if(this.isTurning && noCurvatureInput && !this.hasAngularVelocity()) {
        // register the robot as not turning and set the current angle as the heading lock angle
        this.isTurning = false;
        this.lockCurrentHeading();
        this.headingPID.reset();
        System.out.println("SET PIVOT ANGLE:  " + yaw);
      }

      if(!noCurvatureInput) { // YES curvature input
        // set the robot as turning, so it doesnt lock its heading
        this.isTurning = true;
      }
      
      double rotationInput = this.curvatureSetpoint;

      // if the robot should be locking its heading, and is not turning, 
      // calculate the pid output and set it as the rotationInput
      if(this.shouldHeadingLock && !this.isTurning) {
        rotationInput = this.headingPID.calculate(yaw);
        rotationInput = MathUtil.clamp(rotationInput, -1, 1);
      }

      // set the differentialDrive outputs
      differentialDrive.curvatureDrive(this.speedSetpoint, rotationInput, this.shouldQuickturn);

      // double rotationOutput = this.commandedZRotation;
      SmartDashboard.putNumber("Drivetrain/RotationInputPeriodic", rotationInput);
      SmartDashboard.putNumber("Drivetrain/DriveSpeedPeriodic", this.speedSetpoint);
      // SmartDashboard.putBoolean("Drivetrain/isTurning", this.isTurning);
      SmartDashboard.putNumber("Drivetrain/DriveAngularVelocity", this.angularVelocity);
      SmartDashboard.putNumber("Drivetrain/DriveHeadingAngle", yaw);
      SmartDashboard.putNumber("Drivetrain/DriveAngleSetpoint", this.headingPID.getSetpoint());
      SmartDashboard.putNumber("Drivetrain/shouldHeadingLock", this.shouldHeadingLock ? 1 : -1);
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(imu.getRotation2d(),leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition(), pose);
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

  public void disableHeadingLock() {
    this.shouldHeadingLock = false;
  }

  public void enableHeadingLock() {
    this.shouldHeadingLock = true;
  }

  /**
   * Getter for the angular velocity of the robot in degrees per second
   * 
   * @return Angular velocity for the yaw in degrees per second
   */
  public double getAngularVelocity() { // TODO: Remove
    return this.angularVelocity;
  }

  public boolean hasAngularVelocity() { // TODO: Remove
    return !RobotMath.approximatelyZero(this.getAngularVelocity(), 0.5);
  }

  public void resetAngularVelocity() { // TODO: Remove
    // this.angularVelocityHandler.reset();
  }

  public double getYaw() { // TODO: Remove Use Odometry instead
    return this.imu.getYaw();
  }

  public void stop() {
    this.speedSetpoint = 0.0;
    this.curvatureSetpoint = 0.0;
    this.shouldQuickturn = false;
  }

  public void lockCurrentHeading() {
    this.headingPID.setSetpoint(this.getYaw());
  }

  public double getAngleSetpoint() {
    return this.headingPID.getSetpoint();
  }

  public void setAngleSetpoint(double angle) { // TODO: Remove Use command Framework
    this.headingPID.setSetpoint(RobotMath.constrain180(angle));
  }

  public void turnRelativeAngle(double deltaAngle) { // TODO: Remove Use command Framework
    this.setAngleSetpoint(RobotMath.shiftAngle(
      this.headingPID.getSetpoint(),
      RobotMath.constrain180(deltaAngle)
    ));
    System.out.println(RobotMath.constrain180(deltaAngle));
  }

  // public boolean atAngleSetpoint() { // TODO: Remove
  //   return this.headingPID.atSetpoint();
  // }

  public void resetAnglePID() {
    this.headingPID.reset();
  }

  // public void setSpeed(double speed) { // TODO: Remove
  //   this.xspeed = speed;
  // }

  // public void setCurvature(double curvature) {
  //   this.zrotation = curvature;
  // }

  // public void setQuickturn(boolean quickturn) {
  //   this.allowTurnInPlace = quickturn;
  // }

  public void resetEncoders() { // TODO: Remove create reset odometry class This will cause bugs with the odometry
    leftFrontEncoder.setPosition(0.0);
    rightFrontEncoder.setPosition(0.0);
  }

  private void setupFollowerMotors() {
    // You need to make the rear motors on both sides of the drivetrain follow their respective front motors
    // This is where we will invert motors as needed when we get to testing the drivetrain
    rightRearMotor.follow(rightFrontMotor);
    leftRearMotor.follow(leftFrontMotor);         

    rightFrontMotor.setInverted(Constants.driveports.getRightFrontMotorInversion());
    leftFrontMotor.setInverted(Constants.driveports.getLeftFrontMotorInversion());
  }

  public double getRightEncoderCount() { //TODO: probably remove in favor for odometry
    return (rightFrontEncoder.getPosition() + rightRearEncoder.getPosition()) / 2.0;
  }

  public double getLeftEncoderCount() { //TODO: probably remove in favor for odometry
    return (leftFrontEncoder.getPosition() + leftRearEncoder.getPosition()) / 2.0;
  } 

  //Sets the differential drive using the method curvatureDrive
  public void setCurvatureDrive(double speed, double rotationInput, boolean quickTurn) {
    // System.out.println("" + speed+' '+ rotationInput+' '+ quickTurn);
    SmartDashboard.putNumber("Drivetrain/speed", speed); //TODO: Remove update values in periodic
    SmartDashboard.putNumber("Drivetrain/rotationInput", rotationInput); //TODO: Remove update values in periodic
    this.speedSetpoint = speed;
    this.curvatureSetpoint = rotationInput;
    this.shouldQuickturn = quickTurn;
  }

  public void setRightMotors(double speed) { //TODO: Do we need these methods?
    rightFrontMotor.set(speed);
  }
  public void setLeftMotors(double speed) {//TODO: Do we need these methods?
    leftFrontMotor.set(speed);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }



   /**
     * Simulation Code
     */
    private NavxWrapper simGryo;
    private DifferentialDrivetrainSim m_drivetrainSimulator;
    private RevEncoderSimWrapper leftencsim;
    private RevEncoderSimWrapper rightencsim;
    private boolean simInit = false;

    private void initSim() {
        LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
                Constants.kSimDrivekVLinear,
                Constants.ksimDrivekALinear, Constants.ksimDrivekVAngular,
                Constants.kSimDrivekAAngular);
        m_drivetrainSimulator = new DifferentialDrivetrainSim(
                m_drivetrainSystem, DCMotor.getNEO(2), Constants.WheelConstants.gearRatio, Constants.kSimTrackwidthMeters,
                Units.inchesToMeters(Constants.WheelConstants.wheelDiameterInInches / 2), null);

        // Setup Leader Motors
        this.leftencsim = RevEncoderSimWrapper.create(this.leftFrontMotor);
        this.rightencsim = RevEncoderSimWrapper.create(this.rightFrontMotor);

        // Sim Motors
        simGryo = new NavxWrapper();
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }
        m_drivetrainSimulator.setInputs(
                this.leftFrontMotor.get() * RobotController.getInputVoltage(),
                this.rightFrontMotor.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(Constants.kSimUpdateTime);
        this.leftencsim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        this.leftencsim.setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

        this.rightencsim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        this.rightencsim.setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

        simGryo.getYawGyro().setAngle(-MathUtil.inputModulus(m_drivetrainSimulator.getHeading().getDegrees(), -180, 180)); // TODO add Gyo Vel support
    }
}