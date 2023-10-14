// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.jmhsrobotics.frc2023.commands.arm.CommandArmPitchSimpleDefault;
import org.jmhsrobotics.frc2023.commands.wrist.CommandWristSimpleDefualt;
import org.jmhsrobotics.frc2023.util.DetectRobot;
import org.jmhsrobotics.frc2023.util.sim.BuildDataLogger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private Field2d field = new Field2d();

	private RobotContainer m_robotContainer;
	private boolean lastAutonomous = false;
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// This code must be the first thing that runs otherwise the directory is not
		// updated
		PortForwarder.add(5800, "photonvision.local", 5800);
		if (Robot.isSimulation()) {
			DataLogManager.start(Filesystem.getOperatingDirectory().getAbsolutePath() + "\\logs");
		} else {
			DataLogManager.start();
		}
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		m_robotContainer.getDrivetrain().setBrake(false);
		RobotContainer.getTelemetry().resetIMUYaw();

		// Enables network table logging for data
		DataLogManager.logNetworkTables(true);
		// logs joystick data and driver data
		DriverStation.startDataLog(DataLogManager.getLog());
		SmartDashboard.putData(field);
		BuildDataLogger.LogToNetworkTables();
		BuildDataLogger.LogToWpiLib(DataLogManager.getLog());
		DetectRobot.identifyRobot();
		this.m_robotContainer.getWristSubsystem()
				.setDefaultCommand(new CommandWristSimpleDefualt(this.m_robotContainer.getWristSubsystem()));
		this.m_robotContainer.getArmPitchSubysystem()
				.setDefaultCommand(new CommandArmPitchSimpleDefault(this.m_robotContainer.getArmPitchSubysystem()));
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics that you want ran during disabled, autonomous, teleoperated
	 * and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run(); // TODO: delete
		field.setRobotPose(m_robotContainer.getDrivetrain().getPose());
		// SmartDashboard.putNumber("WristSubsystem/wrist/absolute pitch degrees",
		// m_robotContainer.getArmSubsystem().getArmPitch()
		// + m_robotContainer.getWristSubsystem().getWristPitch());
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		if (!lastAutonomous && Constants.kCoastOnDisable)
			m_robotContainer.getDrivetrain().setBrake(false);
		else {
			m_robotContainer.getDrivetrain().setBrake(true);
		}
		this.m_robotContainer.getDrivetrain().resetHeadingLockPID();
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		// m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
		m_robotContainer.getDrivetrain().setBrake(true);
		// probably shouldn't do this in teleopInit too since that would mess up the
		// calibration
		// during competitions once it transfers from auto to teleop, making it no
		// longer zeroed to the field
		RobotContainer.getTelemetry().resetIMUYaw();
		this.lastAutonomous = true;
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

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

		// this.m_robotContainer.getDrivetrain().resetAngularVelocity();
		this.m_robotContainer.getDrivetrain().lockCurrentHeading();

		this.m_robotContainer.getDrivetrain().resetHeadingLockPID();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
		m_robotContainer.getDrivetrain().setBrake(true);
		this.lastAutonomous = false;
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
