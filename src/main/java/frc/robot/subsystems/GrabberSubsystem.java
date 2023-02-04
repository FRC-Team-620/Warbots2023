package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

	private Solenoid GrabberPiston = new Solenoid(PneumaticsModuleType.REVPH, 43);

	public void setGrabberState(boolean state) {
		GrabberPiston.set(state);
	}
	public boolean getGrabberState() {
		return GrabberPiston.get();
	}
}
