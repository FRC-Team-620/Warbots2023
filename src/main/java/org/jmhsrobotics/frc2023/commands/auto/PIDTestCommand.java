package org.jmhsrobotics.frc2023.commands.auto;

import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PIDTestCommand extends CommandBase {
	private Drivetrain drivetrain;
	// private PIDController drivePID;
	ProfiledPIDController profiledPIDController;
	public PIDTestCommand(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		double maxVelocity = 10;
		double maxAcc = 10;
		SmartDashboard.putNumber("drivetrain/kp", 0.5);
		SmartDashboard.putNumber("drivetrain/plesheading", 0.0);
		Constraints setpointConstraints = new Constraints(10, 10);
		profiledPIDController = new ProfiledPIDController(0.009, 0.003, 0.001, setpointConstraints);
		profiledPIDController.enableContinuousInput(-180, 180);
		// drivePID = new PIDController(0.01, 0, 0);
		// drivePID.enableContinuousInput(-180, 180);
		// drivePID.setP(0.005);
		// SmartDashboard.putData(drivePID);
		SmartDashboard.putData(profiledPIDController);
		SmartDashboard.putNumber("drivetrain/maxV", maxVelocity);
		SmartDashboard.putNumber("drivetrain/maxAcceleration", maxAcc);
		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		// TODO Auto-generated method stub

		double curAngle = drivetrain.getPose().getRotation().getDegrees();
		SmartDashboard.putNumber("drivetrain/plesheading", curAngle);
		SmartDashboard.putNumber("drivetrain/setpoint", profiledPIDController.getSetpoint().position);
		SmartDashboard.putNumber("drivetrain/goal", profiledPIDController.getGoal().position);
		profiledPIDController.setConstraints(new Constraints(SmartDashboard.getNumber("drivetrain/maxV", 10),
				SmartDashboard.getNumber("drivetrain/maxAcceleration", 10)));
		// profiledPIDController.setGoal(new State(50, 0));
		double output = profiledPIDController.calculate(curAngle);
		drivetrain.setCurvatureDrive(0, output, true);
		// SmartDashboard.putNumber("drivetrain/plesheading", curAngle);
		// double output = drivePID.calculate(curAngle);
		// drivetrain.setCurvatureDrive(0, output, true);
		// SmartDashboard.putNumber("drivetrain/outputval", output);
	}

}
