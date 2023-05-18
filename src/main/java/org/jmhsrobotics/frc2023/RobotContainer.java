// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023;

import org.jmhsrobotics.frc2023.Constants.OperatorConstants;
import org.jmhsrobotics.frc2023.commands.ArmSetpointCommand;
// import org.jmhsrobotics.frc2023.commands.ArmCommand;
import org.jmhsrobotics.frc2023.commands.DriveCommand;
import org.jmhsrobotics.frc2023.commands.arm.TelopArmOpenLoop;
import org.jmhsrobotics.frc2023.commands.auto.AutoSelector;
import org.jmhsrobotics.frc2023.commands.auto.CenterChargeStationAuto;
import org.jmhsrobotics.frc2023.commands.gripper.TeleopIntakeOpenLoop;
// import org.jmhsrobotics.frc2023.commands.grabber.ToggleGrabberPitch;
import org.jmhsrobotics.frc2023.commands.vision.AlignPeg;
import org.jmhsrobotics.frc2023.commands.wrist.TeleopWristOpenLoop;
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
import org.jmhsrobotics.frc2023.subsystems.IntakeSubsystem;
// import org.jmhsrobotics.frc2023.subsystems.GrabberMotorSubsystem;
// import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;
import org.jmhsrobotics.frc2023.subsystems.TelemetrySubsystem;
import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDIdleCommand;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import org.jmhsrobotics.frc2023.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
	// public final GrabberSolenoidSubsystem grabberSolenoidSubsystem = new
	// GrabberSolenoidSubsystem();
	// public final GrabberMotorSubsystem grabberMotorSubsystem = new
	// GrabberMotorSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final ArmSubsystem armSubsystem = new ArmSubsystem();
	private final WristSubsystem wristSubsystem = new WristSubsystem();
	// private final ArmSubsystem armSubsystem = new ArmSubsystem();
	// private final GrabberSubsystem grabberSubsystem=new GrabberSubsystem();
	// private final VisionPlaceholder visionPlaceholder = new
	// VisionPlaceholder(drivetrain);
	private AutoSelector autoSelector;

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
			controlBoard.wristControlModifier(),
			controlBoard.override()
		));
		// spotless:on

		// spotless:off
		wristSubsystem.setDefaultCommand(new TeleopWristOpenLoop(
			wristSubsystem, 
			armSubsystem::getArmPitch, 
			controlBoard::wristPitch, 
			controlBoard.wristControlModifier(),
			controlBoard.override()));
		// spotless:on

		// grabberMotorSubsystem.setDefaultCommand(new InstantCommand(() -> {
		// grabberMotorSubsystem.setGrabberMotor(MathUtil.clamp(
		// controlBoard.intakeWheels() +
		// (grabberSolenoidSubsystem.getGrabberPitchState() ? 0.05 : 0.0), -1,
		// 1));
		// }, grabberMotorSubsystem));

		// TODO: make this not hellish (plz)

		// spotless:off
		intakeSubsystem.setDefaultCommand(
			new TeleopIntakeOpenLoop(
				intakeSubsystem, 
				Constants.kGripperType, 
				controlBoard::intakeWheels, 
				armSubsystem::getScoringType
			)
		);
		// spotless:on

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
		// controlBoard.changeScoringType().onTrue(new InstantCommand(() ->
		// armSubsystem.switchScoringType()));

		controlBoard.armPresetStowed()
				.onTrue(new ArmSetpointCommand(Constants.Setpoints.STOWED, Constants.kGripperType, this));

		controlBoard.armPresetFloor()
				.onTrue(new ArmSetpointCommand(Constants.Setpoints.FLOOR, Constants.kGripperType, this));

		controlBoard.armPresetMid()
				.onTrue(new ArmSetpointCommand(Constants.Setpoints.MID, Constants.kGripperType, this));

		controlBoard.armPresetHigh()
				.onTrue(new ArmSetpointCommand(Constants.Setpoints.HIGH, Constants.kGripperType, this));

		// controlBoard.armPresetPickup()
		// .onTrue(new ArmSetpointCommand(Constants.Setpoints.PICKUP,
		// Constants.kGripperType, this));

		// controlBoard.closeGrabber().onTrue(new ToggleIntakePiston(intakeSubsystem));

		// controlBoard.armWrist().onTrue(new
		// ToggleGrabberPitch(grabberSolenoidSubsystem));

		// controlBoard.autoBalance().onTrue(new AutoBalance(drivetrain, false,
		// ledSubsystem));

		// controlBoard.toggleHeadingLock().onTrue(
		// new InstantCommand(() ->
		// this.drivetrain.setHeadingLock(!this.getDrivetrain().getShouldHeadingLock())));

		// controlBoard.getDriver().a().onTrue(new TurnAngle(drivetrain, 180.0));

		// controlBoard.getDriver().x().onTrue(new TurnDeltaAngle(drivetrain, 90.0));

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

	public double getAbsoluteStow() {
		return getArmSubsystem().getArmPitch() + 130;
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

	public IntakeSubsystem getIntakeSubsystem() {
		return intakeSubsystem;
	}

	public ArmSubsystem getArmSubsystem() {
		return armSubsystem;
	}

	public WristSubsystem getWristSubsystem() {
		return wristSubsystem;
	}

	public AutoSelector getAutoSelector() {
		return autoSelector;
	}

	public static TelemetrySubsystem getTelemetry() {
		return RobotContainer.telemetry;
	}

	public ControlBoard getControlBoard() {
		return controlBoard;
	}
}
