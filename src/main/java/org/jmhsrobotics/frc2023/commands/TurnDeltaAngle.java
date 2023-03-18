package org.jmhsrobotics.frc2023.commands;

import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.RobotMath;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

public class TurnDeltaAngle extends TurnAngle {

	public TurnDeltaAngle(Drivetrain drivetrain, Supplier<Double> deltaAngleSupplier) {
		super(drivetrain, () -> RobotMath.shiftAngle(drivetrain.getAngleSetpoint(), deltaAngleSupplier.get()));
	}

	public TurnDeltaAngle(Drivetrain drivetrain, double deltaAngle) {
		super(drivetrain, () -> RobotMath.shiftAngle(drivetrain.getAngleSetpoint(), deltaAngle));
	}
}

// spotless:off
/*
import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.RobotContainer;
import org.jmhsrobotics.frc2023.RobotMath;
import org.jmhsrobotics.frc2023.Constants.TurnAngleCommandConstants;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// A DriveCommand command that uses the Drivetrain subsystem.
public class TurnDeltaAngle extends InstantCommand {

	// public TurnDeltaAngle(Drivetrain drivetrain, double relativeAngle) {
	// super(() -> drivetrain.turnRelativeAngle(relativeAngle));
	// }

	private Drivetrain drivetrain;
	private double deltaAngle;

	private ProfiledPIDController profiledAnglePID;

	public TurnDeltaAngle(Drivetrain drivetrain, double deltaAngle) {

		this.drivetrain = drivetrain;
		this.deltaAngle = RobotMath.constrain180(deltaAngle);

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
    	// spotless:o // TODO: Should be "...on" once this code is uncommented

		this.profiledAnglePID.enableContinuousInput(-180, 180);
		this.profiledAnglePID.setTolerance(2, TurnAngleCommandConstants.maxAngularVelocity);

		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		double finalAngle = RobotMath.shiftAngle(this.drivetrain.getAngleSetpoint(), this.deltaAngle);

		this.profiledAnglePID.reset(RobotContainer.getTelemetry().getYaw());
		this.profiledAnglePID.setGoal(finalAngle);

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
*/
// spotless:on
