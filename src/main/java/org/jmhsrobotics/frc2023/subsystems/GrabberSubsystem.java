package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

	private Solenoid grabberPiston = new Solenoid(PneumaticsModuleType.REVPH,
			Constants.driveports.getIntakeSolenoidId());
	private CANSparkMax grabberMotor = new CANSparkMax(Constants.driveports.getIntakeCANId(), MotorType.kBrushless);

	public GrabberSubsystem() {
		grabberMotor.setSmartCurrentLimit(40);
		grabberMotor.setIdleMode(IdleMode.kBrake);
	}

	public void setGrabberState(boolean state) {
		grabberPiston.set(state);
	}
	public boolean getGrabberState() {
		return grabberPiston.get();
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
