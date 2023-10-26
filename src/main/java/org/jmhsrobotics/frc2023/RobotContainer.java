// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023;

import org.jmhsrobotics.frc2023.Constants.OperatorConstants;
import org.jmhsrobotics.frc2023.commands.AutoDriveDistance;
// import org.jmhsrobotics.frc2023.commands.ArmCommand;
import org.jmhsrobotics.frc2023.commands.DriveCommand;
import org.jmhsrobotics.frc2023.commands.arm.CommandArmPitchSimple;
import org.jmhsrobotics.frc2023.commands.auto.AutoScoreAndBalance;
import org.jmhsrobotics.frc2023.commands.gripper.TeleopIntakeOpenLoop;
import org.jmhsrobotics.frc2023.commands.wrist.CommandSetGripperOpen;
import org.jmhsrobotics.frc2023.commands.wrist.CommandWristSimple;
import org.jmhsrobotics.frc2023.commands.wrist.PistonIntakeCommand;
import org.jmhsrobotics.frc2023.oi.CompControl;
// import org.jmhsrobotics.frc2023.commands.auto.AutoSelector;
// import org.jmhsrobotics.frc2023.commands.grabber.ToggleGrabberPitch;
// import org.jmhsrobotics.frc2023.commands.auto.AutoSelector;
import org.jmhsrobotics.frc2023.oi.ControlBoard;
import org.jmhsrobotics.frc2023.subsystems.ArmExtensionSubsystem;
import org.jmhsrobotics.frc2023.subsystems.ArmPitchSubsystem;
// import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;
// import org.jmhsrobotics.frc2023.subsystems.GrabberMotorSubsystem;
// import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;
import org.jmhsrobotics.frc2023.subsystems.TelemetrySubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import org.jmhsrobotics.frc2023.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	private static final TelemetrySubsystem telemetry = new TelemetrySubsystem();

	// The robot's subsystems and commands are defined here...
	private final ControlBoard controlBoard;
	private final CommandXboxController driver = new CommandXboxController(OperatorConstants.driverControllerPort);
	private final CommandXboxController operator = new CommandXboxController(OperatorConstants.operatorControllerPort);

	private final Drivetrain drivetrain = new Drivetrain();
	private final LEDSubsystem ledSubsystem = new LEDSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final WristSubsystem wristSubsystem = new WristSubsystem();
	private final ArmExtensionSubsystem armExtensionSubsystem = new ArmExtensionSubsystem();
	private final ArmPitchSubsystem armPitchSubsystem = new ArmPitchSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		SmartDashboard.putData(CommandScheduler.getInstance());
		controlBoard = new CompControl();
		configureBindings();
		// Setting up default command which is a command that runs every time no other
		// command that uses that subsystem is running
		drivetrain.setDefaultCommand(new DriveCommand(drivetrain, controlBoard));
		// SmartDashboard.putData("MoveArm", new
		// CommandArmPitchSimple(this.armPitchSubsystem, -50));
		SmartDashboard.putData("MoveWrist", new CommandWristSimple(this.wristSubsystem, 0.3));
		// SmartDashboard.putData("AutoBalance", new AutoBalance(this.drivetrain,
		// false));
		SmartDashboard.putData("scoreing auto", new AutoScoreAndBalance(this.drivetrain, this.wristSubsystem,
				this.intakeSubsystem, this.armPitchSubsystem));
		SmartDashboard.putData("openIntakePiston", new CommandSetGripperOpen(this.intakeSubsystem, true));
		SmartDashboard.putData("auto drive", new AutoDriveDistance(this.drivetrain, 30));
		// TODO: make this not hellish (plz)

		// spotless:off
		this.intakeSubsystem.setDefaultCommand(
				new TeleopIntakeOpenLoop(
						intakeSubsystem,
						controlBoard::intakeWheels,
						this.controlBoard));

		controlBoard.switchGrabber().onTrue(new PistonIntakeCommand(this.intakeSubsystem));

	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
	 * for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
	 * Flight joysticks}.
	 */
	private void configureBindings() {

		// stow position
		this.controlBoard.armPresetStowed()
				.onTrue(new ParallelCommandGroup(new CommandWristSimple(this.wristSubsystem, NewConstants.WRIST_PRESET_STOWED),
						new CommandArmPitchSimple(this.armPitchSubsystem, NewConstants.ARM_PRESET_STOWED)));
		// floor pick up position
		this.controlBoard.armPresetFloor()
				.onTrue(new ParallelCommandGroup(new CommandWristSimple(this.wristSubsystem, NewConstants.WRIST_RESET_FLOOR),
						new CommandArmPitchSimple(this.armPitchSubsystem, NewConstants.ARM_PRESET_FLOOR)));
		// mid position
		this.controlBoard.armPresetMid()
				.onTrue(new ParallelCommandGroup(new CommandWristSimple(this.wristSubsystem, NewConstants.WRIST_PRESET_MID),

						new CommandArmPitchSimple(this.armPitchSubsystem, NewConstants.ARM_PRESET_MID)));

		// station pickup setpoint
		this.controlBoard.armPresetSlide()
				.onTrue(new ParallelCommandGroup(new CommandWristSimple(this.wristSubsystem, NewConstants.WRIST_PRESET_STATION),
						new CommandArmPitchSimple(this.armPitchSubsystem, NewConstants.ARM_PRESET_STATION)));

		// cube pickup setpoint
		this.controlBoard.cubePickUp().onTrue(new ParallelCommandGroup(new CommandWristSimple(this.wristSubsystem, NewConstants.WRIST_PRESET_FLOOR_CUBE),
				new CommandArmPitchSimple(this.armPitchSubsystem, NewConstants.ARM_PRESET_FLOOR)));
	}

	public Drivetrain getDrivetrain() {
		return this.drivetrain;
	}

	public LEDSubsystem getLEDSubsystem() {
		return this.ledSubsystem;
	}

	public ControlBoard getControls() {
		return this.controlBoard;
	}

	public static TelemetrySubsystem getTelemetry() {
		return RobotContainer.telemetry;
	}

	public ControlBoard getControlBoard() {
		return controlBoard;
	}

	public ArmPitchSubsystem getArmPitchSubysystem() {
		return this.armPitchSubsystem;
	}

	public WristSubsystem getWristSubsystem() {
		return this.wristSubsystem;
	}

	public IntakeSubsystem getIntakeSubsystem() {
		return this.intakeSubsystem;
	}
}
