package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

	private Solenoid grabberIntakePiston = new Solenoid(42, PneumaticsModuleType.REVPH, 1); // Constants.driveports.getIntakeSolenoidId()
	private Solenoid grabberPitchPiston = new Solenoid(42, PneumaticsModuleType.REVPH, 0);
	private CANSparkMax grabberMotor = new CANSparkMax(Constants.driveports.getIntakeCANId(), MotorType.kBrushless);

	public GrabberSubsystem() {
		grabberMotor.setSmartCurrentLimit(40);
		grabberMotor.setIdleMode(IdleMode.kBrake);
	}

	public void setGrabberIntakeState(boolean state) {
		grabberIntakePiston.set(state);
	}
	public boolean getGrabberIntakeState() {
		return grabberIntakePiston.get();
	}

	public void setGrabberPitchState(boolean state) {
		grabberPitchPiston.set(state);
	}
	public boolean getGrabberPitchState() {
		return grabberPitchPiston.get();
	}

	public void setGrabberMotor(double speed) {
		this.grabberMotor.set(speed);
	}

	public void wheelForward() {
		this.grabberMotor.set(1);
	}

	public void wheelBackward() {
		this.grabberMotor.set(-1);
	}

	public void stopGrabberWheel() {
		this.grabberMotor.set(0);
	}
}
