// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023;

import org.jmhsrobotics.frc2023.Constants.ArmConstants;
import org.jmhsrobotics.frc2023.Constants.OperatorConstants;
// import org.jmhsrobotics.frc2023.commands.ArmCommand;
import org.jmhsrobotics.frc2023.commands.DriveCommand;
import org.jmhsrobotics.frc2023.commands.TelopArmOpenLoop;
import org.jmhsrobotics.frc2023.commands.arm.CommandArmExtension;
import org.jmhsrobotics.frc2023.commands.arm.CommandArmPitch;
import org.jmhsrobotics.frc2023.commands.arm.CommandArmExtensionThenPitch;
import org.jmhsrobotics.frc2023.commands.arm.CommandArmPitchThenExtension;
import org.jmhsrobotics.frc2023.commands.auto.AutoBalance;
import org.jmhsrobotics.frc2023.commands.auto.AutoSelector;
import org.jmhsrobotics.frc2023.commands.auto.CenterChargeStationAuto;
import org.jmhsrobotics.frc2023.commands.grabber.CommandGrabberSolenoids;
import org.jmhsrobotics.frc2023.commands.grabber.ToggleGrabberIntake;
import org.jmhsrobotics.frc2023.commands.grabber.ToggleGrabberPitch;
import org.jmhsrobotics.frc2023.commands.vision.AlignPeg;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.commands.auto.AutoSelector;
import org.jmhsrobotics.frc2023.commands.auto.CenterChargeStationAuto;
import org.jmhsrobotics.frc2023.commands.vision.AlignPeg;
import org.jmhsrobotics.frc2023.oi.CompControl;
import org.jmhsrobotics.frc2023.oi.ControlBoard;
import org.jmhsrobotics.frc2023.oi.SelectableControlBoard;
import org.jmhsrobotics.frc2023.oi.SingleControl;
// import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.subsystems.GrabberMotorSubsystem;
import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;
import org.jmhsrobotics.frc2023.subsystems.TelemetrySubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDIdleCommand;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import org.jmhsrobotics.frc2023.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

	public final Drivetrain drivetrain = new Drivetrain();
	public final LEDSubsystem ledSubsystem = new LEDSubsystem();
	public final GrabberSolenoidSubsystem grabberSolenoidSubsystem = new GrabberSolenoidSubsystem();
	public final GrabberMotorSubsystem grabberMotorSubsystem = new GrabberMotorSubsystem();
	public final ArmSubsystem armSubsystem = new ArmSubsystem();
	// private final ArmSubsystem armSubsystem = new ArmSubsystem();
	// private final GrabberSubsystem grabberSubsystem=new GrabberSubsystem();
	// private final VisionPlaceholder visionPlaceholder = new
	// VisionPlaceholder(drivetrain);
	public AutoSelector autoSelector;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		SmartDashboard.putData(CommandScheduler.getInstance());
		SelectableControlBoard selectable = new SelectableControlBoard("competition", new CompControl());
		selectable.addOption("single", new SingleControl());
		controlBoard = selectable;
		// Configure the trigger bindings
		configureBindings();
		// Setting up default command which is a command that runs every time no other
		// command that uses that subsystem is running
		drivetrain.setDefaultCommand(new DriveCommand(drivetrain, controlBoard));

		// OpenLoop Control!
		// spotless:off
		armSubsystem.setDefaultCommand(new TelopArmOpenLoop(
			armSubsystem, 
			controlBoard::armPitch, 
			controlBoard::armExtend, 
			controlBoard.overrideTeleopArm()
		));
		// spotless:on

		// grabberMotorSubsystem.setDefaultCommand(new InstantCommand(() -> {
		// grabberMotorSubsystem.setGrabberMotor(MathUtil.clamp(
		// controlBoard.intakeWheels() +
		// (grabberSolenoidSubsystem.getGrabberPitchState() ? 0.05 : 0.0), -1,
		// 1));
		// }, grabberMotorSubsystem));

		// TODO: make this not hellish (plz)
		grabberMotorSubsystem.setDefaultCommand(new InstantCommand(() -> {
			grabberMotorSubsystem.setGrabberMotor(MathUtil.clamp(
					((armSubsystem.getScoringType() == ArmSubsystem.scoringType.CUBE && controlBoard.intakeWheels() > 0)
							? 0.4
							: 1.0) * controlBoard.intakeWheels()
							+ (grabberSolenoidSubsystem.getGrabberPitchState() ? 0.05 : 0.0),
					-1, 1));
		}, grabberMotorSubsystem));

		// grabberMotorSubsystem.setGrabberMotor(controlBoard.intakeWheels()),
		// grabberMotor));
		// Closed Loop
		// armSubsystem.setDefaultCommand(new TeleopArm(armSubsystem,
		// operator::getLeftY, operator::getRightY));

		// driver.getHID()));

		// spotless:off
		ledSubsystem.setDefaultCommand(new LEDIdleCommand(
				ledSubsystem, 
				armSubsystem::isCone,
				armSubsystem::getTeleopWasEnded
			)
		);
		// spotless:on

		autoSelector = new AutoSelector(this);
		SmartDashboard.putData(new AlignPeg(drivetrain));
		SmartDashboard.putData(new CenterChargeStationAuto(drivetrain, ledSubsystem));
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
		// Triggers are a thing that we might need to use so keep that in mind
		// driver.b().onTrue(new ArmCommand(armSubsystem, driver.getHID()));
		// driver.leftBumper().onTrue(
		// new InstantCommand(() ->
		// grabberSubsystem.setGrabberState(!grabberSubsystem.getGrabberState())));
		// driver.start().onTrue(new AutoDriveDistance(drivetrain, -3));

		// driver.y().onTrue(new TurnDeltaAngle(drivetrain, 90));
		// driver.y().onTrue(new TurnDeltaAngle(drivetrain, 180));
		// driver.a().onTrue(new CenterChargeStationAuto(drivetrain, ledSubsystem));
		// driver.x().onTrue(new AutoBalance(drivetrain, true, ledSubsystem));
		// driver.b().onTrue(new AlignPeg(drivetrain));

		// operator.x().onTrue(new CommandArm(armSubsystem, 0.2, 0));
		// operator.b().onTrue(new CommandArm(armSubsystem, 0, 180));
		// controlBoard.alignPeg().onTrue(new AlignPeg(drivetrain));
		controlBoard.changeScoringType().onTrue(new InstantCommand(() -> armSubsystem.setScoringType()));
		controlBoard.armPresetStowed().onTrue(new ConditionalCommand(
				new ParallelCommandGroup(
						new CommandArmExtensionThenPitch(armSubsystem, 0.0, ArmConstants.stowedDegrees,
								controlBoard.overrideTeleopArm()),
						new CommandGrabberSolenoids(grabberSolenoidSubsystem, false, false)),
				new ParallelCommandGroup(
						new CommandArmExtensionThenPitch(armSubsystem, 0.0, ArmConstants.stowedDegrees,
								controlBoard.overrideTeleopArm()),
						new CommandGrabberSolenoids(grabberSolenoidSubsystem, true, false)),
				armSubsystem::isCone));
		// new SequentialCommandGroup(new ParallelCommandGroup(
		// new CommandArmExtension(armSubsystem, 0.0, controlBoard.overrideTeleopArm()),
		// new InstantCommand(() -> {
		// grabberSolenoidSubsystem.setGrabberIntakeState(false);
		// grabberSolenoidSubsystem.setGrabberPitchState(false);
		// }, grabberSolenoidSubsystem)),
		// new CommandArm(armSubsystem, 0.0, ArmConstants.stowedDegrees,
		// controlBoard.overrideTeleopArm())),
		// new SequentialCommandGroup(new ParallelCommandGroup(
		// new CommandArmExtension(armSubsystem, 0.0, controlBoard.overrideTeleopArm()),
		// new InstantCommand(() -> {
		// grabberSolenoidSubsystem.setGrabberIntakeState(true);
		// grabberSolenoidSubsystem.setGrabberPitchState(false);
		// }, grabberSolenoidSubsystem)), new CommandArm(armSubsystem, 0.0,
		// ArmConstants.stowedDegrees, controlBoard.overrideTeleopArm())),

		controlBoard.armPresetFloor().onTrue(new ConditionalCommand(

				new SequentialCommandGroup(new CommandArmPitch(armSubsystem, 35, controlBoard.overrideTeleopArm()),
						new ParallelCommandGroup(
								new CommandArmExtension(armSubsystem, 0.8, controlBoard.overrideTeleopArm()),
								new CommandGrabberSolenoids(grabberSolenoidSubsystem, false, true))),

				new SequentialCommandGroup(new CommandArmPitch(armSubsystem, 35, controlBoard.overrideTeleopArm()),
						new ParallelCommandGroup(
								new CommandArmExtension(armSubsystem, 0.8, controlBoard.overrideTeleopArm()),
								new CommandGrabberSolenoids(grabberSolenoidSubsystem, true, true))),

				armSubsystem::isCone));

		// new CommandArm(armSubsystem, ArmConstants.maxExtensionLengthMillims, 55,
		// controlBoard.overrideTeleopArm()),
		// new CommandArm(armSubsystem, ArmConstants.maxExtensionLengthMillims, 55,
		// controlBoard.overrideTeleopArm()),
		// armSubsystem::isCone));

		// controlBoard.armPresetFloor().whileTrue(new InstantCommand(() ->
		// grabberMotorSubsystem.setGrabberMotor(-1), armSubsystem));
		// controlBoard.armPresetFloor().onFalse(new InstantCommand(() ->
		// grabberMotorSubsystem.setGrabberMotor(0), armSubsystem));

		controlBoard.armPresetMid()
				.onTrue(new ConditionalCommand(
						new CommandArmPitchThenExtension(armSubsystem, 1.0, 95, controlBoard.overrideTeleopArm()),
						new CommandArmPitchThenExtension(armSubsystem, 1.0, 80, controlBoard.overrideTeleopArm()),
						// new SequentialCommandGroup(new CommandArmPitch(armSubsystem, 95,
						// controlBoard.overrideTeleopArm()),
						// new CommandArmExtension(armSubsystem, 1.0,
						// controlBoard.overrideTeleopArm())),
						// new SequentialCommandGroup(new CommandArmPitch(armSubsystem, 80,
						// controlBoard.overrideTeleopArm()),
						// new CommandArmExtension(armSubsystem, 1.0,
						// controlBoard.overrideTeleopArm())),
						armSubsystem::isCone));

		controlBoard.armPresetHigh()
				.onTrue(new ConditionalCommand(
						new CommandArmPitchThenExtension(armSubsystem, 1.0, 247, controlBoard.overrideTeleopArm()),
						new CommandArmPitchThenExtension(armSubsystem, 1.0, 247, controlBoard.overrideTeleopArm()),
						// new SequentialCommandGroup(new CommandArmPitch(armSubsystem, 247,
						// controlBoard.overrideTeleopArm()),
						// new CommandArmExtension(armSubsystem, 1.0,
						// controlBoard.overrideTeleopArm())),
						// new SequentialCommandGroup(new CommandArmPitch(armSubsystem, 247,
						// controlBoard.overrideTeleopArm()),
						// new CommandArmExtension(armSubsystem, 1.0,
						// controlBoard.overrideTeleopArm())),
						armSubsystem::isCone));

		// controlBoard.armPresetHigh()
		// .onTrue(new ConditionalCommand(
		// new CommandArm(armSubsystem, ArmConstants.maxExtensionLengthMillims, 247,
		// controlBoard.overrideTeleopArm()),
		// new CommandArm(armSubsystem, ArmConstants.maxExtensionLengthMillims, 247,
		// controlBoard.overrideTeleopArm()),
		// armSubsystem::isCone));/

		controlBoard.armPresetPickup()
				.onTrue(new ConditionalCommand(
						new ParallelCommandGroup(
								new CommandArmExtensionThenPitch(armSubsystem, 0.0, 67,
										controlBoard.overrideTeleopArm()),
								new CommandGrabberSolenoids(grabberSolenoidSubsystem, false, true)),
						new ParallelCommandGroup(
								new CommandArmExtensionThenPitch(armSubsystem, 0.0, 67,
										controlBoard.overrideTeleopArm()),
								new CommandGrabberSolenoids(grabberSolenoidSubsystem, true, true)),
						armSubsystem::isCone));

		controlBoard.closeGrabber().onTrue(new ToggleGrabberIntake(grabberSolenoidSubsystem));

		controlBoard.armWrist().onTrue(new ToggleGrabberPitch(grabberSolenoidSubsystem));

		controlBoard.autoBalance().onTrue(new AutoBalance(drivetrain, false, ledSubsystem));

		controlBoard.toggleHeadingLock().onTrue(
				new InstantCommand(() -> this.drivetrain.setHeadingLock(!this.getDrivetrain().getShouldHeadingLock())));

		// driver.povLeft().onTrue(new CommandArm(armSubsystem, .5, 0));
		// driver.povUp().onTrue(new CommandArm(armSubsystem, 1, 45));
		// driver.povDown().onTrue(new CommandArm(armSubsystem, 0, -45));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		// return Autos.taxi(drivetrain);
		return autoSelector.getCommand();
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
}
