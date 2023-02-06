// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023;

import org.jmhsrobotics.frc2023.Constants.OperatorConstants;
// import org.jmhsrobotics.frc2023.commands.ArmCommand;
import org.jmhsrobotics.frc2023.commands.DriveCommand;
// import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.util.LEDs.LEDIdleCommand;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;

// import org.jmhsrobotics.frc2023.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2023.commands.DriveStraight;
import org.jmhsrobotics.frc2023.commands.TurnDeltaAngle;
import org.jmhsrobotics.frc2023.commands.auto.AutoSelector;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final CommandXboxController driver = new CommandXboxController(OperatorConstants.driverControllerPort);
	public final Drivetrain drivetrain = new Drivetrain();
	public final LEDSubsystem ledSubsystem = new LEDSubsystem();
	// private final ArmSubsystem armSubsystem = new ArmSubsystem();
	// private final GrabberSubsystem grabberSubsystem=new GrabberSubsystem();
	public AutoSelector autoSelector;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		// Setting up default command which is a command that runs every time no other
		// command that uses that subsystem is running
		drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driver));
		ledSubsystem.setDefaultCommand(new LEDIdleCommand(ledSubsystem, drivetrain));
		// armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem));
		autoSelector = new AutoSelector(this);
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
		// driver.b().onTrue(new ArmCommand(armSubsystem));
		// driver.leftBumper().onTrue(new InstantCommand(() ->
		// grabberSubsystem.setGrabberState(!grabberSubsystem.getGrabberState())));
		// driver.x().onTrue(new AutoDriveDistance(drivetrain, 100));

		// driver.y().onTrue(new TurnDeltaAngle(drivetrain, 90));
		driver.y().onTrue(new TurnDeltaAngle(drivetrain, 180));

		driver.x().onTrue(new DriveStraight(drivetrain, 2));
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
}
