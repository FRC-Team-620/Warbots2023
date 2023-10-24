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
		// SmartDashboard.putNumber("IntakeSubsystem/intake/piston state",
		// this.getIntakePistonState() ? 1 : -1);
		// SmartDashboard.putBoolean("IntakeSubsystem/inkate/piston state",
		// intakePiston.get());
		// SmartDashboard.putNumber("IntakeSubsystem/intake/speed",
		// this.intakeMotor.get());
	}

	// ****** INTAKE MOTOR ******

	public void setIntakeMotor(double speed) {
		if (speed > -.1 && speed <= .1) {
			speed = -.1;
		}
		this.intakeMotor.set(speed);
	}

	public void setIntakeMotor(Direction dir, double speed) {
		switch (dir) {
			case FORWARD :
			case IN :
				this.setIntakeMotor(-speed);
				break;
			case REVERSE :
			case OUT :
				this.setIntakeMotor(speed);
				break;
		}
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
		// state = button (pressed/not pressed)
		// intake.set(true/false)
		// if (state) {
		// this.pistonOpen = !this.pistonOpen;
		// }
		// this.pistonOpen = state ? !this.pistonOpen : this.pistonOpen;
		this.pistonOpen = !pistonOpen;
		this.intakePiston.set(this.pistonOpen);
	}

	public boolean getIntakePistonState() {
		return false;
		// return this.intakePiston.get();
	}

	// public void toggleIntakePistonState() {
	// this.switchIntakePistonState(!this.getIntakePistonState());
	// }
}
