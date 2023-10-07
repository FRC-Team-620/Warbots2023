package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends SubsystemBase {
	private CANSparkMax telescopeMotor = new CANSparkMax(Constants.driveports.getArmExtensionCANId(),
			MotorType.kBrushless);
	private RelativeEncoder extensionEncoder = telescopeMotor.getEncoder();

	public ArmExtensionSubsystem() {
		this.extensionEncoder.setPosition(0);
		// Testing
		this.telescopeMotor.setIdleMode(IdleMode.kCoast);
		telescopeMotor.setSmartCurrentLimit(40);
		telescopeMotor.setIdleMode(IdleMode.kCoast);
	}

	public void setSpeed(double speed) {
		this.telescopeMotor.set(speed);
	}

	public void resetEncoder(double position) {
		this.extensionEncoder.setPosition(position);
	}

	public double getEncoderPostition() {
		return this.extensionEncoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("ArmExtension/position", getEncoderPostition());
	}
}
