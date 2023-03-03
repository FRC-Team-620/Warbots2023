package org.jmhsrobotics.frc2023.oi;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SelectableControlBoard implements ControlBoard {
	private SendableChooser<ControlBoard> selector;
	public SelectableControlBoard(String name, ControlBoard control) {
		this.selector = new SendableChooser<>();
		setDefualtOption(name, control);
		/// SendableChooser<CommandBase> autoSelector = new SendableChooser<>();
		SmartDashboard.putData("Control selector", selector);
	}

	public void setDefualtOption(String name, ControlBoard control) {
		this.selector.setDefaultOption(name, control);
	}

	public void addOption(String name, ControlBoard control) {
		selector.addOption(name, control);
	}

	@Override
	public double driveForward() {
		return selector.getSelected().driveForward();
	}

	@Override
	public boolean isDriveSlow() {
		return selector.getSelected().isDriveSlow();
	}

	@Override
	public double driveTurn() {
		return selector.getSelected().driveTurn();
	}

	@Override
	public boolean isQuickTurn() {
		return selector.getSelected().isQuickTurn();
	}

	@Override
	public double armPitch() {
		return selector.getSelected().armPitch();
	}

	@Override
	public double armExtend() {
		return selector.getSelected().armExtend();
	}

	@Override
	public Trigger armPresetFloor() {
		return selector.getSelected().armPresetFloor();
	}

	@Override
	public Trigger armPresetMid() {
		return selector.getSelected().armPresetMid();
	}

	@Override
	public Trigger armPresetHigh() {
		return selector.getSelected().armPresetHigh();
	}

	@Override
	public Trigger armStop() {
		return selector.getSelected().armStop();
	}

	@Override
	public Trigger Intake() {
		return selector.getSelected().Intake();
	}

	@Override
	public Trigger armWrist() {
		return selector.getSelected().armWrist();
	}

	@Override
	public Trigger alignPeg() {
		return selector.getSelected().alignPeg();
	}

	@Override
	public Trigger armPresetStowed() {
		// TODO Auto-generated method stub
		return selector.getSelected().armPresetStowed();
	}

	@Override
	public Trigger armPresetPickup() {
		// TODO Auto-generated method stub
		return selector.getSelected().armPresetPickup();
	}

	@Override
	public double intakeWheels() {
		// TODO Auto-generated method stub
		return selector.getSelected().intakeWheels();
	}
	@Override
	public Trigger closeGrabber(){
		return selector.getSelected().closeGrabber();
	}

	@Override
	public Trigger autoBalance(){
		return selector.getSelected().autoBalance();
	}

}
