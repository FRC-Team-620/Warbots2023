package org.jmhsrobotics.frc2023.commands;

import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.RobotContainer;
import org.jmhsrobotics.frc2023.Constants.TurnAngleCommandConstants;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A DriveCommand command that uses the Drivetrain subsystem. */
public class TurnAngle extends CommandBase {

	// public TurnDeltaAngle(Drivetrain drivetrain, double relativeAngle) {
	// super(() -> drivetrain.turnRelativeAngle(relativeAngle));
	// }

	private Drivetrain drivetrain;
	private Supplier<Double> angleSupplier;

	private ProfiledPIDController profiledAnglePID;

	public TurnAngle(Drivetrain drivetrain, Supplier<Double> angleSupplier) {

		this.drivetrain = drivetrain;
		this.angleSupplier = angleSupplier;
		// this.angle = RobotMath.constrain180(angle);

		// spotless:off
		this.profiledAnglePID = new ProfiledPIDController(
      		Constants.driveports.getTurnDeltaAnglePID().kp,
			Constants.driveports.getTurnDeltaAnglePID().ki,
			Constants.driveports.getTurnDeltaAnglePID().kd,
      		new Constraints(
				TurnAngleCommandConstants.maxAngularVelocity, 
        		TurnAngleCommandConstants.maxAngularAcceleration
			)
    	);
    	// spotless:on

		this.profiledAnglePID.enableContinuousInput(-180, 180);
		this.profiledAnglePID.setTolerance(2, TurnAngleCommandConstants.maxAngularVelocity);

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		this.profiledAnglePID.reset(RobotContainer.getTelemetry().getYaw());
		this.profiledAnglePID.setGoal(this.angleSupplier.get());

		this.drivetrain.resetHeadingLockPID();
		this.drivetrain.disableHeadingLock();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		double rotation = this.profiledAnglePID.calculate(RobotContainer.getTelemetry().getYaw());

		this.drivetrain.setCurvatureDrive(0, rotation, true);

		SmartDashboard.putNumber("TurnDeltaAngle/setpoint.position", this.profiledAnglePID.getSetpoint().position);
		SmartDashboard.putNumber("TurnDeltaAngle/setpoint.velocity", this.profiledAnglePID.getSetpoint().velocity);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

		this.drivetrain.resetHeadingLockPID();
		this.drivetrain.stop();
		this.drivetrain.enableHeadingLock();
		this.drivetrain.lockCurrentHeading();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// System.out.println("SETPOINT: " + this.drivetrain.atAngleSetpoint());
		// System.out.println("ANG VEL: " + this.drivetrain.hasAngularVelocity());
		// System.out.println(this.drivetrain.getAngularVelocity());
		// return this.drivetrain.atAngleSetpoint() &&
		// !this.drivetrain.hasAngularVelocity();
		return this.profiledAnglePID.atGoal();
	}
}
