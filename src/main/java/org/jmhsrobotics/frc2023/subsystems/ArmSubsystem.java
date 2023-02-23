package org.jmhsrobotics.frc2023.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
	// Set the motors which power the basic functions of the arm
	private Compressor phCompressor = new Compressor(42, PneumaticsModuleType.REVPH);
	private Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 42);
	private CANSparkMax pitchMotor = new CANSparkMax(5, MotorType.kBrushless);
	private CANSparkMax telescopeMotor = new CANSparkMax(6, MotorType.kBrushless);

	// todo 1.make ports in own file

	public void setSolenoid(boolean state) {
		solenoid.set(state);
	}

	public boolean getSolenoid() {
		return solenoid.get();
	}
	// Sets the motor controlling arm height
	public void setPitchMotor(double amount) {
		pitchMotor.set(amount);
	}
	// Sets the motor controlling arm length
	public void setTelescopeMotor(double amount) {
		telescopeMotor.set(amount);
	}
}
