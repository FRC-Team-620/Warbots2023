// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.vision.AlignPeg;
import frc.robot.util.DetectRobot;
import frc.robot.util.sim.BuildDataLogger;
import frc.robot.util.sim.SimPipeLineVisionSystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Field2d field = new Field2d();
  // SimVisionSystem ssys ,ssys2;
  SimPipeLineVisionSystem simPhoton;
  private RobotContainer m_robotContainer;
  private boolean lastAutonomous = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
   
    if(Robot.isSimulation()){
      DataLogManager.start(Filesystem.getOperatingDirectory().getAbsolutePath() + "\\logs");
    }else{
      DataLogManager.start();
    }
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // PhotonCamera.setVersionCheckEnabled(!Robot.isSimulation());
    m_robotContainer = new RobotContainer();
    m_robotContainer.getDrivetrain().setBrake(false);
   
    
    // Enables network table logging for data 
    DataLogManager.logNetworkTables(true);
    // logs joystick data and driver data 
    DriverStation.startDataLog(DataLogManager.getLog());
    SmartDashboard.putData(field);
    
    loadAprilTags();

    BuildDataLogger.LogToNetworkTables();
    BuildDataLogger.LogToWpiLib(DataLogManager.getLog());
    DetectRobot.identifyRobot();
    PortForwarder.add(5800, "photonvision.local", 5800);
    PortForwarder.add(1181, "photonvision.local", 1181);
    PortForwarder.add(1182, "photonvision.local", 1182);
    PortForwarder.add(1183, "photonvision.local", 1183);
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(new AlignPeg(m_robotContainer.drivetrain));
  }
  private void putPose3d(String key, Pose3d... value){
    double[] data = new double[value.length * 7];
    for (int i = 0; i < value.length; i++) {
      data[i * 7] = value[i].getX();
      data[i * 7 + 1] = value[i].getY();
      data[i * 7 + 2] = value[i].getZ();
      data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
      data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
      data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
      data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
    }
    SmartDashboard.putNumberArray(key, data);
  }
  private void loadAprilTags(){
    try {
      AprilTagFieldLayout layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getAbsolutePath() + "/2023-chargedup.json");

      SimVisionTarget target = new SimVisionTarget(new Pose3d(6,6,1.3, new Rotation3d(25,25,25)), 0.2, 0.2, 0);

      simPhoton = new SimPipeLineVisionSystem("mainCam", 90, new Transform3d(new Translation3d(0,0,1), new Rotation3d()), 20, 640, 480, 10);
      simPhoton.getPipeline(0).addVisionTargets(layout);
      simPhoton.getPipeline(simPhoton.addPipeline()).addSimVisionTarget(target);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    updateField();

  }

  private void updateField(){
    field.setRobotPose(m_robotContainer.drivetrain.getPose()); // Updates robot's position on field2d

    this.simPhoton.processFrame(m_robotContainer.drivetrain.getPose());

    var result = PhotonManager.getInstance().mainCam.getLatestResult(); //TODO: handle switch between sim and real world.
    var ids = new ArrayList<Double>();
    var tagpos = new ArrayList<Pose2d>();
    ArrayList<Pose3d> tag3d = new ArrayList<Pose3d>();
    if(result.hasTargets()){
      for(var target: result.getTargets()){
        ids.add(target.getFiducialId() + 0.0);
        tagpos.add(new Pose3d(m_robotContainer.drivetrain.getPose()).plus(target.getBestCameraToTarget()).toPose2d());
        tag3d.add(new Pose3d(m_robotContainer.drivetrain.getPose().getX(), m_robotContainer.drivetrain.getPose().getY(), 1, new Pose3d( m_robotContainer.drivetrain.getPose()).getRotation()).plus(target.getBestCameraToTarget()));
      }
    }
    SmartDashboard.putNumberArray("TAGIDS",  ids.toArray(new Double[ids.size()]));
    putPose3d("3dtags",  tag3d.toArray(new Pose3d[tag3d.size()]));
    field.getObject("Tags").setPoses(tagpos); //Display simulated tag positions
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (lastAutonomous && Constants.kCoastOnDisable)
      m_robotContainer.getDrivetrain().setBrake(false);
    else{
      m_robotContainer.getDrivetrain().setBrake(true);
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.getDrivetrain().setBrake(true);
    this.lastAutonomous = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getDrivetrain().setBrake(true);
    this.lastAutonomous = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getDrivetrain().setBrake(true);
    this.lastAutonomous = false;
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {  
    

  }

   
   
}
