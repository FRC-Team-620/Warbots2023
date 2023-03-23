package org.jmhsrobotics.frc2023.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSolenoidSubsystem extends SubsystemBase {

	private Solenoid grabberIntakePiston = new Solenoid(42, PneumaticsModuleType.REVPH, 0); // Constants.driveports.getIntakeSolenoidId()
	private Solenoid grabberPitchPiston = new Solenoid(42, PneumaticsModuleType.REVPH, 1);

	public GrabberSolenoidSubsystem() {

	}

	public void setGrabberIntakeState(boolean state) {
		grabberIntakePiston.set(state);
	}
	public boolean getGrabberIntakeState() {
		return grabberIntakePiston.get();
	}
	public void toggleIntake() {
		this.setGrabberIntakeState(!this.getGrabberIntakeState());
	}

	public void setGrabberPitchState(boolean state) {
		grabberPitchPiston.set(state);
	}
	public boolean getGrabberPitchState() {
		return grabberPitchPiston.get();
	}
	public void togglePitch() {
		this.setGrabberPitchState(!this.getGrabberPitchState());
	}
}
