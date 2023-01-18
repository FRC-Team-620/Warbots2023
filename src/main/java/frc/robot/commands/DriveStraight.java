package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStraight extends CommandBase {

    private Drivetrain drivetrain;
    private double startingAngle;

    private PIDController distancePID; 

    public DriveStraight(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.distancePID = new PIDController(
            Constants.DriveConstants.kPDriveDistance,
            Constants.DriveConstants.kIDriveDistance,
            Constants.DriveConstants.kDDriveDistance
        );
        this.distancePID.setTolerance(0.02, 0.5);

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}