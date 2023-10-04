package org.jmhsrobotics.frc2023.commands.wrist;

import org.jmhsrobotics.frc2023.subsystems.WristSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandWristSimple extends CommandBase {
	WristSubsystem wristSubsystem;
	double speed;
	CommandXboxController driver;

	public CommandWristSimple(WristSubsystem wristSubsystem, double speed, CommandXboxController driver) {
		this.wristSubsystem = wristSubsystem;
		this.speed = speed;
		this.driver = driver;
		addRequirements(wristSubsystem);
	}

	@Override
	public void execute() {
		if ((wristSubsystem.getWristPitch() < 0.8 && wristSubsystem.getWristPitch() > 0.1)) {
			this.wristSubsystem.setWristMotor(MathUtil.applyDeadband(driver.getRightY(), 0.1));
			System.out.println("mark");
		} else {
			this.wristSubsystem.setWristMotor(0);
		}
		// this.wristSubsystem.setPitch(this.angle.get()); // clamps the input
	}

}
