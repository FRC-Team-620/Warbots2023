package org.jmhsrobotics.frc2023.commands;

import org.jmhsrobotics.frc2023.subsystems.ArmSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopArm extends CommandBase {

	private ArmSubsystem armSubsystem;
	private Supplier<Double> pitchSpeed;
	private Supplier<Double> linearSpeed;
	private final double maxPitchSpeedDegreesSec = 45;
	private final double maxExtensionSpeedMPS = 0.25;

	// Constructor
	// Takes in a angle for the arm pitch and a distance for the linear joint
	// (Prizmatic joint)
	public TeleopArm(ArmSubsystem armSubsystem, Supplier<Double> pitchSpeed, Supplier<Double> linearSpeed) {
		addRequirements(armSubsystem);
		this.armSubsystem = armSubsystem;
		this.pitchSpeed = pitchSpeed;
		this.linearSpeed = linearSpeed;
	}

	// Only runs when the command is scedualed
	// Are you good?
	@Override
	public void initialize() {

	}
	// execute runs every 20ms while the command is running
	@Override
	public void execute() {
		// Lets say this is 0.5
		// So this the controler output when the command is run IE new
		// TelopArm(armsubsystem, controler::getleftX,0)
		double driverDemandedSpeed = pitchSpeed.get(); // GEts the controlers depemanded speed
		double driverDemandedExtension = linearSpeed.get();

		// The arm is currenty set at 90 dregrees
		// armSubsystem.getArmPitch()
		SmartDashboard.putNumber("arm_info/driverDemandedSpeed", driverDemandedSpeed);

		// 100
		// 100
		// armSubsystem.setArmExtension(armspeed); // ABSOLUTE VALUES
		// // Looks perfect
		// Next step would be to bind the controls in RobotContainer.java
		// Making a comand would look somthing like this new TelopArm(armsubsystem,
		// controler::getleftX,controler::getleftY)
		// Make sure to set the TelopARm as the default command for the subsystem to
		// that it auto runs when everything starts up
		// Does that make sense?
		// armSubsystem.setDefaultCommand(new TelopArm());
		SmartDashboard.putNumber("arm_info/driverDemandedExtension", driverDemandedExtension);
		double inputArmExtension = MathUtil.applyDeadband(linearSpeed.get(), 0.2);
		if (inputArmExtension != 0) {
			armSubsystem.setArmExtension(armSubsystem.getArmLength() + (inputArmExtension * maxExtensionSpeedMPS));
		}

		double inputArmPitch = MathUtil.applyDeadband(pitchSpeed.get(), 0.2);
		if (inputArmPitch != 0) {
			armSubsystem.setArmPitch(armSubsystem.getArmPitch() + (driverDemandedSpeed * maxPitchSpeedDegreesSec));
		}
		// armSubsystem.setArmExtension(linearSpeed.get());

		// This needs to be done inside of the Robot container. otherwise you would need
		// to first run the command to set it as the default.
		// got it
	}

	// IS called once when the command finishes or is intruppted or otherwise ended
	// for any reason.
	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			armSubsystem.stop();
		}
	}

	// isFinished run every 20ms and ends the command once it returns true.
	@Override
	public boolean isFinished() {
		// return armSubsystem.atPitchGoal() && armSubsystem.atExtensionGoal();
		return false;
	}
}
