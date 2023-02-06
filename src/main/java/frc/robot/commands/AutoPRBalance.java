package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMath;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoPRBalance extends CommandBase {

    private Drivetrain drivetrain;

    private RobotMath.DiminishingAverageHandler pitchHandler;
    private RobotMath.DiminishingAverageHandler rollHandler;

    private PIDController pitchSpeedPID;
    private PIDController rollCurvaturePID;

    private AutoPRBalance(Drivetrain drivetrain) {

        this.drivetrain = drivetrain;

        this.pitchHandler = new RobotMath.DiminishingAverageHandler(2.0);
        this.rollHandler = new RobotMath.DiminishingAverageHandler(2.0);

        this.pitchSpeedPID = new PIDController(
            AutoConstants.kPPitchSpeed,
            AutoConstants.kIPitchSpeed,
            AutoConstants.kDPitchSpeed  
        );

        this.pitchSpeedPID.setSetpoint(0.0);
        this.pitchSpeedPID.setTolerance(
            0.3, 
            40.0
        );

        this.rollCurvaturePID = new PIDController(
            AutoConstants.kPRollCurvature,
            AutoConstants.kIRollCurvature,
            AutoConstants.kDRollCurvature
        );

        this.rollCurvaturePID.setSetpoint(0.0);
        this.rollCurvaturePID.setTolerance(
            0.3,
            40.0
        );
        
    }

    @Override
    public void initialize() {

        this.pitchHandler.reset();
        this.rollHandler.reset();

        this.pitchSpeedPID.reset();
        this.rollCurvaturePID.reset();

        this.drivetrain.disableHeadingLock();
    }

    @Override
    public void execute() {

    }
    
    @Override
    public void end(boolean interrupted) {

        this.drivetrain.stop();

        this.drivetrain.enableHeadingLock();
    }
}
