package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPitchSubsystem extends SubsystemBase {
	private CANSparkMax pitchMotor = new CANSparkMax(Constants.driveports.getArmAngleCANId(), MotorType.kBrushless);
	private RelativeEncoder pitchEncoder = pitchMotor.getEncoder();

	public ArmPitchSubsystem() {
		this.pitchEncoder.setPosition(0);
		// Testing
		this.pitchMotor.setSmartCurrentLimit(40);
		this.pitchMotor.setIdleMode(IdleMode.kCoast);
	}

	public void setSpeed(double speed) {
		this.pitchMotor.set(speed);
	}

	public void resetEncoder(double position) {
		this.pitchEncoder.setPosition(position);
	}

	public double getEncoderPostition() {
		return this.pitchEncoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("ArmPitch/angle", this.getEncoderPostition());
	}
}
