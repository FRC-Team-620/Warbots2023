package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStraight extends CommandBase {

    private Drivetrain drivetrain;
    private PIDController distancePID; 

    private double distance;

    public DriveStraight(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;

        this.distance = distance;

        this.distancePID = new PIDController(
            Constants.driveports.getkPDriveDistance(),
            Constants.driveports.getkIDriveDistance(),
            Constants.driveports.getkDDriveDistance()
        );
        this.distancePID.setTolerance(0.04, 0.1);

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.distancePID.setSetpoint(this.drivetrain.getLeftEncoderCount() + distance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double current = this.drivetrain.getLeftEncoderCount();
        // this.drivetrain.setSpeed(this.distancePID.calculate(current));
        this.drivetrain.setCurvatureDrive(this.distancePID.calculate(current), 0, false);

        SmartDashboard.putNumber("distance setpoint", this.distancePID.getSetpoint());
        SmartDashboard.putNumber("distance current", drivetrain.getLeftEncoderCount()); 
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.distancePID.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.distancePID.atSetpoint();
    }
    
}