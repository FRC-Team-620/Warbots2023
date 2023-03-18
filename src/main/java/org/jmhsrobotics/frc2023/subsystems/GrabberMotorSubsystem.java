package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberMotorSubsystem extends SubsystemBase {

	private CANSparkMax grabberMotor = new CANSparkMax(Constants.driveports.getIntakeCANId(), MotorType.kBrushless);

	public GrabberMotorSubsystem() {
		grabberMotor.setSmartCurrentLimit(25);
		grabberMotor.setIdleMode(IdleMode.kBrake);
		// grabberMotor.setInverted(true);
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
