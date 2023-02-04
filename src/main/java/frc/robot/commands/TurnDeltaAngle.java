package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotMath;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TurnAngleCommandConstants;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An DriveCommand command that uses an Drivetrain subsystem. */
public class TurnDeltaAngle extends InstantCommand {

  // public TurnDeltaAngle(Drivetrain drivetrain, double relativeAngle) {
  //   super(() -> drivetrain.turnRelativeAngle(relativeAngle));
  // }

  private Drivetrain drivetrain;
  private double deltaAngle;

  private ProfiledPIDController profiledAnglePID;

  public TurnDeltaAngle(Drivetrain drivetrain, double deltaAngle) {

    this.drivetrain = drivetrain;
    this.deltaAngle = RobotMath.constrain180(deltaAngle);

    this.profiledAnglePID = new ProfiledPIDController(
      TurnAngleCommandConstants.kPTurnAngle, 
      TurnAngleCommandConstants.kITurnAngle, 
      TurnAngleCommandConstants.kDTurnAngle, 
      new Constraints(
          TurnAngleCommandConstants.maxAngularVelocity, 
          TurnAngleCommandConstants.maxAngularAcceleration // This is the limiting factor
      )
    );

    this.profiledAnglePID.enableContinuousInput(-180, 180);
    this.profiledAnglePID.setTolerance(
      2, 
      TurnAngleCommandConstants.maxAngularVelocity
    );

    addRequirements(drivetrain);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      double finalAngle = RobotMath.shiftAngle(
        this.drivetrain.getAngleSetpoint(),
        this.deltaAngle
      );

      this.profiledAnglePID.reset(this.drivetrain.getYaw());
      this.profiledAnglePID.setGoal(finalAngle);

      this.drivetrain.resetAnglePID();
      this.drivetrain.disableHeadingLock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotation = this.profiledAnglePID.calculate(
      this.drivetrain.getYaw()
    );

    this.drivetrain.setCurvatureDrive(0, rotation, true);

    SmartDashboard.putNumber("TurnDeltaAngle/setpoint.position", this.profiledAnglePID.getSetpoint().position);
    SmartDashboard.putNumber("TurnDeltaAngle/setpoint.velocity", this.profiledAnglePID.getSetpoint().velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.drivetrain.resetAnglePID();
    this.drivetrain.stop();
    this.drivetrain.enableHeadingLock();
    this.drivetrain.lockCurrentHeading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("SETPOINT: " + this.drivetrain.atAngleSetpoint());
    // System.out.println("ANG VEL:  " + this.drivetrain.hasAngularVelocity());
    // System.out.println(this.drivetrain.getAngularVelocity());
    // return this.drivetrain.atAngleSetpoint() && !this.drivetrain.hasAngularVelocity();
    return this.profiledAnglePID.atGoal();
  }
}
