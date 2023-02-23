// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023;

import org.jmhsrobotics.frc2023.Constants.OperatorConstants;
import org.jmhsrobotics.frc2023.commands.AutoDriveDistance;
import org.jmhsrobotics.frc2023.commands.CommandArm;
// import org.jmhsrobotics.frc2023.commands.ArmCommand;
import org.jmhsrobotics.frc2023.commands.DriveCommand;
import org.jmhsrobotics.frc2023.commands.TeleopArm;
import org.jmhsrobotics.frc2023.commands.TurnDeltaAngle;
import org.jmhsrobotics.frc2023.commands.auto.AutoBalance;
import org.jmhsrobotics.frc2023.commands.auto.AutoSelector;
import org.jmhsrobotics.frc2023.commands.vision.AlignPeg;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
// import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.subsystems.TelemetrySubsystem;
import org.jmhsrobotics.frc2023.subsystems.GrabberSubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import org.jmhsrobotics.frc2023.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2023.commands.TurnDeltaAngle;
import org.jmhsrobotics.frc2023.commands.auto.AutoBalance;
import org.jmhsrobotics.frc2023.commands.auto.AutoSelector;
import org.jmhsrobotics.frc2023.commands.auto.CenterChargeStationAuto;
import org.jmhsrobotics.frc2023.commands.vision.AlignPeg;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

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
	private final CommandXboxController driver = new CommandXboxController(OperatorConstants.driverControllerPort);
	private final CommandXboxController operator = new CommandXboxController(OperatorConstants.operatorControllerPort);

	public final Drivetrain drivetrain = new Drivetrain();
	public final LEDSubsystem ledSubsystem = new LEDSubsystem();
	public final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
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
		// Configure the trigger bindings
		configureBindings();
		// Setting up default command which is a command that runs every time no other
		// command that uses that subsystem is running
		drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driver));
		// armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem,
		armSubsystem.setDefaultCommand(new TeleopArm(armSubsystem, operator::getLeftX, operator::getLeftY));
		// driver.getHID()));

		// spotless:off
		// ledSubsystem.setDefaultCommand(new LEDIdleCommand(
		// 		ledSubsystem, 
		// 		() -> drivetrain.getIsTurning()
		// 	)
		// );
		// spotless:on

		autoSelector = new AutoSelector(this);
		SmartDashboard.putData(new AlignPeg(drivetrain));
		SmartDashboard.putData(new CenterChargeStationAuto(drivetrain));
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
		driver.start().onTrue(new AutoDriveDistance(drivetrain, -3));

		// driver.y().onTrue(new TurnDeltaAngle(drivetrain, 90));
		driver.y().onTrue(new TurnDeltaAngle(drivetrain, 90));

		driver.x().onTrue(new AutoBalance(drivetrain, true));
		driver.b().onTrue(new AlignPeg(drivetrain));

		driver.povLeft().onTrue(new CommandArm(armSubsystem, .5, 0));
		driver.povUp().onTrue(new CommandArm(armSubsystem, 1, 45));
		driver.povDown().onTrue(new CommandArm(armSubsystem, 0, -45));
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

	public static TelemetrySubsystem getTelemetry() {
		return RobotContainer.telemetry;
	}
}
