package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.Constants.Direction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

	// intake hardware
	private CANSparkMax intakeMotor = new CANSparkMax(Constants.driveports.getIntakeCANId(), MotorType.kBrushed);
	private Solenoid intakePiston = new Solenoid(42, PneumaticsModuleType.REVPH, 0); // Constants.driveports.getIntakeSolenoidId()
	private boolean pistonOpen = true;
	public IntakeSubsystem() {

		this.intakeMotor.setSmartCurrentLimit(20);
		this.intakeMotor.setIdleMode(IdleMode.kBrake);
		this.intakeMotor.setInverted(true);
	}

	@Override
	public void periodic() {

		SmartDashboard.putNumber("IntakeSubsystem/intake/speed", this.getIntakeSpeed());
	}

	// ****** INTAKE MOTOR ******

	public void setIntakeMotor(double speed) {
		// dumb way to check if any of the triggers are pressed
		if (speed > -.1 && speed <= .1) {
			// if no triggers are pressed, spein the intake motor at 10% its full speed
			speed = -.1;
		} else if (pistonOpen) {
			// triggers are pressed and piston is open, run intake wheels in 30% of its full speed
			speed *= 0.3;
		}
		this.intakeMotor.set(speed);
	}

	public double getIntakeSpeed() {
		return this.intakeMotor.get();
	}

	public void stopIntakeMotor() {
		this.intakeMotor.set(0.1);
	}

	public void intakeOut() {
		setIntakeMotor(-1);
	}

	// ****** INTAKE PISTON ******

	public void switchIntakePistonState() {
		this.pistonOpen = !pistonOpen;
		this.intakePiston.set(this.pistonOpen);
	}

	public boolean getIntakePistonState() {
		return this.intakePiston.get();
	}
	public void setIntakePistonState(boolean open) {
		System.out.println("TEST");
		this.pistonOpen = open;
		this.intakePiston.set(this.pistonOpen);
	}
}
