package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An DriveCommand command that uses an Drivetrain subsystem. */
public class TurnDeltaAngle extends CommandBase {

    private Drivetrain drivetrain;
    private double deltaAngle;

    public TurnDeltaAngle(Drivetrain drivetrain, double deltaAngle) {
      this.drivetrain = drivetrain;
      this.deltaAngle = deltaAngle;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(drivetrain);    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.drivetrain.turnRelativeAngle(this.deltaAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      if(interrupted) System.out.println("INTERRUPTED");
      this.drivetrain.resetAnglePID();
      System.out.println("TURN COMMAND ENDED");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      // System.out.println(this.drivetrain.getAngularVelocity());
      return this.drivetrain.atAngleSetpoint() && !this.drivetrain.hasAngularVelocity();
    }
}
