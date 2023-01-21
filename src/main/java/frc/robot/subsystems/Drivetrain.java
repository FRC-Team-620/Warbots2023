// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants;
// import frc.robot.Constants.CANIdsTestBot;
//import edu.wpi.first.wpilibj.*;
import frc.robot.Constants.WheelConstants;
import frc.robot.util.sim.NavxWrapper;
import frc.robot.util.sim.RevEncoderSimWrapper;

public class Drivetrain extends SubsystemBase {
  
  // I'm giving you a basic structure and you guys job is to fill it out
  // Hopefully you will learn something from this process and that it will help you with being able to do it on your own
  // Just giving you a helping hand for your first time round

  // Device id's are CAN pin numbers and you will be seeing a lot more of them in the future so I suggest you get used to it 
  // Second argument is a Enum and the long and short of it is it's words that represent a number in a way that makes it more readable, but in this case it's just idenifing that the motor we have plugged into that CAN slot is a brushless motor
  private CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveports.leftFrontMotorCANId, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveports.rightFrontMotorCANId, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(Constants.driveports.leftRearMotorCANId, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(Constants.driveports.rightRearMotorCANId, MotorType.kBrushless);

  private RelativeEncoder leftFrontEncoder;
  private RelativeEncoder rightFrontEncoder;
  private AHRS navx;
  private DifferentialDriveOdometry odometry;
  
  public double getHeading() { // TODO: Remove Use Odom class
    return navx.getAngle();
  }

  private DifferentialDrive differentialDrive;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    setupMotors();
    setupFollowerMotors();
    initSensors();
    //Setup differential drive with left front and right front motors as the parameters for the new DifferentialDrive
    differentialDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);
  }
  private void setupMotors() {
    leftFrontMotor = setupMotor(leftFrontMotor);
    rightFrontMotor = setupMotor(rightFrontMotor);
    leftRearMotor = setupMotor(leftRearMotor);
    rightRearMotor = setupMotor(rightRearMotor);
  }

  private void initSensors() {
    navx = new AHRS(Port.kMXP);
    leftFrontEncoder = leftFrontMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();
    leftFrontEncoder.setPositionConversionFactor(WheelConstants.conversionFactor);
    leftFrontEncoder.setVelocityConversionFactor(WheelConstants.conversionFactor);

    rightFrontEncoder.setPositionConversionFactor(WheelConstants.conversionFactor);
    rightFrontEncoder.setVelocityConversionFactor(WheelConstants.conversionFactor);
    
    odometry = new DifferentialDriveOdometry(navx.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
}
  
  @Override
  public void periodic() {
      odometry.update(navx.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
      SmartDashboard.putNumber("Heading", navx.getYaw());
      
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

  

  //Sets the differential drive using the method curvatureDrive
  public void setCurvatureDrive(double speed, double rotationInput, boolean quickTurn) {
    differentialDrive.curvatureDrive(speed, rotationInput, quickTurn);
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

        simGryo.getYawGyro().setAngle(-m_drivetrainSimulator.getHeading().getDegrees()); // TODO add Gyo Vel support
    }
}
