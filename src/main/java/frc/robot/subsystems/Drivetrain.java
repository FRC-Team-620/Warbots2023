// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.commons.lang3.NotImplementedException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
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

  private Timer antiSnapBackTimer = new Timer();
  private boolean isAntiSnapback = false;
  
  private PIDController headingPID;
  private DifferentialDriveOdometry odometry;
  
  private double commandedXSpeed = 0.0;
  private double commandedZRotation = 0.0;
  private boolean commandedAllowTurnInPlace = false;
  
  private double angularVelocity = 0.0;

  private boolean headingLock = false;
  
  public double getHeading() { // TODO: Remove Use Odom class
    return imu.getAngle();//Could use getYaw
  }
  public double getPitch(){
    return imu.getPitch();
  }
  public void resetGyro(){
    return;
  }

  private double setAngle;
  private boolean isTurning = false;
  // private int tickAccumulation = 0;
  private double previousAngle;

  private RobotMath.DiminishingAverageHandler angularVelocityHandler;
  private DifferentialDrive differentialDrive;
  private boolean isInRamsete;
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
    // headingPID.setSetpoint(0.0); // IMPORTANT
    headingPID.setTolerance(2,1);


    //this.angularVelocityHandler = new RobotMath.DiminishingAverageHandler(2);
    //Setup differential drive with left front and right front motors as the parameters for the new DifferentialDrive
    differentialDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);
  }
  public void setBrake(boolean brake){
    IdleMode mode;
    if (brake){
      mode = IdleMode.kBrake;
    }
    else{
      mode = IdleMode.kCoast;
    }
    
    leftFrontMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    leftRearMotor.setIdleMode(mode);
    rightRearMotor.setIdleMode(mode);
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
      odometry.update(imu.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
      double yaw = imu.getYaw();
      SmartDashboard.putNumber("Drivetrain/Heading", yaw);
      if (this.isInRamsete){
        return;
      }
      // System.out.println(leftFrontEncoder.getPosition());
      // double relativeChange = RobotMath.relativeAngle(this.previousAngle, yaw);

      // this.angularVelocity = this.angularVelocityHandler.feed(
      //   relativeChange / RobotConstants.secondsPerTick
      // );
      // this.previousAngle = yaw;

      // boolean noCurvatureInput = RobotMath.approximatelyZero(curvatureSetpoint);

      // if(this.isTurning && noCurvatureInput && !this.hasAngularVelocity()) {

      //   this.setAngle = yaw;
      //   this.isTurning = false;
      //   this.headingPID.reset();
      //   // this.tickAccumulation = 0;
      //   System.out.println("SET PIVOT ANGLE:  " + yaw);
      // }

      // if(!noCurvatureInput) { // YES curvature input
      //   this.isTurning = true;
      // } else if(this.isTurning) { // no curvature input, isTurning is true
      //   // this.tickAccumulation++;
      // }

      // double rotationInput = this.curvatureSetpoint;

      // if(!this.isTurning) { // NOT TURNING
      //   double relativeAngle = RobotMath.relativeAngle(this.setAngle, yaw);
      //   rotationInput = this.headingPID.calculate(relativeAngle);
      //   SmartDashboard.putNumber("relative_angle", relativeAngle);
      // }

      double rotationOutput = this.commandedZRotation;
      SmartDashboard.putNumber("Drivetrain/RotationInputPeriodic", this.commandedZRotation);
      SmartDashboard.putNumber("Drivetrain/RotationOutputPeriodic", rotationOutput);
      SmartDashboard.putBoolean("Drivetrain/HeadingLock", headingLock);
      // If the robot is not turning and if we are not already in a heading lock then turn on heading lock and set the target
      // angle to the current heading
      
      if (rotationOutput == 0 && headingLock == false) {

        // headingLock = true;
        // headingPID.reset();//Prevent integral weirdness
        // headingPID.setSetpoint(yaw);
        if (!isAntiSnapback) {
          antiSnapBackTimer.start();
        }

        if (antiSnapBackTimer.get() > 2) {
          antiSnapBackTimer.stop();
          antiSnapBackTimer.reset();
          isAntiSnapback = false;
          headingLock = true;
          headingPID.reset();//Prevent integral weirdness
          headingPID.setSetpoint(imu.getYaw());
        } else {
          isAntiSnapback = true;
          headingPID.setSetpoint(imu.getYaw());
          rotationOutput = MathUtil.clamp(headingPID.calculate(imu.getYaw()), -1, 1);
        }
      }

      // Disengage heading lock if bot is turning
      if (this.commandedZRotation != 0) {
        headingLock = false;
        isAntiSnapback = false;
        antiSnapBackTimer.stop();
        antiSnapBackTimer.reset();
      }

      if (headingLock) {
        // Locks the value in the proper range for curvature drive
        double calculate = headingPID.calculate(yaw);
        rotationOutput = MathUtil.clamp(calculate, -1, 1);
      }

      


      SmartDashboard.putNumber("Drivetrain/RotationOutput", rotationOutput);
      differentialDrive.curvatureDrive(this.commandedXSpeed, rotationOutput, this.commandedAllowTurnInPlace);

      SmartDashboard.putNumber("Drivetrain/angular_velocity", this.angularVelocity);
      SmartDashboard.putNumber("Drivetrain/set_angle", setAngle);
      SmartDashboard.putNumber("Drivetrain/heading_angle", yaw);
  }

  public void resetOdometry(Pose2d pose){
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
    this.commandedXSpeed = 0.0;
    this.commandedZRotation = 0.0;
    this.commandedAllowTurnInPlace = false;
  }

  public void setCurrentAngle(double angle) { // TODO: Remove Use command Framework
    this.setAngle = angle;
  }

  public void turnRelativeAngle(double deltaAngle) { // TODO: Remove Use command Framework
    this.setCurrentAngle(RobotMath.shiftAngle(this.setAngle, deltaAngle));
  }

  public boolean atAngleSetpoint() { // TODO: Remove
    return this.headingPID.atSetpoint();
  }

  public void resetAnglePID() { // TODO: Remove
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

  public SequentialCommandGroup createRamseteCommand(Trajectory path) {
    final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(AutoConstants.trackWidthMeters);
    
		return new InstantCommand(() -> {this.isInRamsete = true;}).andThen( new RamseteCommand(
			path,
			this::getPose,
			new RamseteController(),
			new SimpleMotorFeedforward(
				AutoConstants.trajectoryFeedForwardVolts,
				AutoConstants.trajectoryFeedForwardVoltSecondsPerMeter,
				AutoConstants.trajectoryFeedForwardVoltSecondsSquaredPerMeter
			),
			driveKinematics,
			this::getWheelSpeeds,
			new PIDController(AutoConstants.trajectorykP, AutoConstants.trajectorykI, AutoConstants.trajectorykD),
			new PIDController(AutoConstants.trajectorykP, AutoConstants.trajectorykI, AutoConstants.trajectorykD),
			this::tankDriveVolts,
			this
		)).andThen(new InstantCommand(() -> {this.isInRamsete = false;}));
	}
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(
			leftFrontEncoder.getVelocity(),
			rightFrontEncoder.getVelocity()
		);
	}
  //Sets the differential drive using the method curvatureDrive
  public void setCurvatureDrive(double speed, double rotationInput, boolean quickTurn) {
    // System.out.println("" + speed+' '+ rotationInput+' '+ quickTurn);
    SmartDashboard.putNumber("Drivetrain/speed", speed); //TODO: Remove update values in periodic
    SmartDashboard.putNumber("Drivetrain/rotationInput", rotationInput); //TODO: Remove update values in periodic
    this.commandedXSpeed = speed;
    this.commandedZRotation = rotationInput;
    this.commandedAllowTurnInPlace = quickTurn;
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
		leftFrontMotor.setVoltage(leftVolts);
		rightFrontMotor.setVoltage(-rightVolts);
		// drive.feed();
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