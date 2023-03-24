package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

	// intake hardware
	private CANSparkMax intakeMotor = new CANSparkMax(Constants.driveports.getIntakeCANId(), MotorType.kBrushless);
	private Solenoid intakePiston = new Solenoid(42, PneumaticsModuleType.REVPH, 0); // Constants.driveports.getIntakeSolenoidId()

	public IntakeSubsystem() {

		this.intakeMotor.setSmartCurrentLimit(20);
		this.intakeMotor.setIdleMode(IdleMode.kBrake);
		this.intakeMotor.setInverted(true);
	}

	@Override
	public void periodic() {

		SmartDashboard.putNumber("IntakeSubsystem/intake/speed", this.getIntakeSpeed());
		SmartDashboard.putNumber("IntakeSubsystem/intake/piston state", this.getIntakePistonState() ? 1 : -1);
	}

	// ****** INTAKE MOTOR ******

	public void setIntakeMotor(double speed) {
		this.intakeMotor.set(speed);
	}

	public double getIntakeSpeed() {
		return this.intakeMotor.get();
	}

	public void stopIntakeMotor() {
		this.intakeMotor.set(0.0);
	}

	// ****** INTAKE PISTON ******

	public void setIntakePistonState(boolean state) {
		this.intakePiston.set(state);
	}

	public boolean getIntakePistonState() {
		return this.intakePiston.get();
	}

	public void toggleIntakePistonState() {
		this.setIntakePistonState(!this.getIntakePistonState());
	}

}
