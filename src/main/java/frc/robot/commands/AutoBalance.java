package frc.robot.commands;

//import org.apache.commons.io.filefilter.FalseFileFilter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMath;
import frc.robot.Constants.ArenaConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotMath.DiminishingAverageHandler;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
    private Drivetrain drivetrain;
    private double moveLimit;
    private DiminishingAverageHandler robotPitchHandler;
    private PIDController pidController;
    private boolean isBalancing = false;
    private boolean onChargeStation = false;
    private Pose2d chargeCenterPosition;
    private Pose2d balancedPosition; 
    private boolean atLimit = false;
    private Pose2d limitPosition;

    public AutoBalance(Drivetrain drivetrain, boolean backwards){
        this.drivetrain = drivetrain;
        robotPitchHandler = new DiminishingAverageHandler(0.5);
        pidController = new PIDController(AutoConstants.autoDistanceKP, AutoConstants.autoDistanceKI, AutoConstants.autoDistanceKD);
    }
    @Override
    public void initialize(){
        this.moveLimit = ArenaConstants.kchargeStationLengthMeters/2;
    }

    @Override
    public void execute(){
        // Command flow: 
        // Command started with the back of the robot facing the charge station 
        // (for possible future localization using apriltags)
        //
        // Robot should drive backwards until the robot reaches an angle that 
        // tells whether it is on the charge station (charge station side sections are 11 degrees, 
        // middle is 15 degrees when tipped)
        // 
        // The robot should then record its position, to make sure it doesn't drive back off the other side.
        //
        // The robot should then creep backwards if it is tipped forwards, 
        // and forwards if it is tipped backwards, until it reaches roughly level. 
        // (there is a high threshold for level in this code as we don't want the robot moving while it is in motion)
        //
        // If it reaches level, it should activate a PID to hold that position, unless it stops being level. 
        // 
        // If it reaches a a limit, it should hold its position until it either needs to move the other direction, 
        // or the robot is level, at which point it will also hold position. 
        //
        // This command will never complete manually, as it needs to hold its position. 

        double pitch = this.robotPitchHandler.feed(drivetrain.getPitch());

        if (!hasReachedChargeStation()){
            this.drivetrain.setCurvatureDrive(-1 * AutoConstants.balanceCreepSpeed, 0, false);
        }
        else{
            if (RobotMath.approximatelyZero(pitch, AutoConstants.balancedAngle)){
                if (!this.isBalancing){
                    this.isBalancing = true;
                    this.atLimit = false;
                    this.balancedPosition = this.drivetrain.getPose();
                    this.pidController.reset();
                    this.pidController.setSetpoint(0);
                }
                this.drivetrain.setCurvatureDrive(this.pidController.calculate(getRelativeDistance(this.balancedPosition)), 0, false);
    
            }
            else{
                this.isBalancing = false;
                
                if (pitch < 0 && getRelativeDistance(chargeCenterPosition) > moveLimit){
                    this.drivetrain.setCurvatureDrive(-1 * AutoConstants.balanceCreepSpeed, 0, false);
                }
                else if (pitch > 0 && getRelativeDistance(chargeCenterPosition) < moveLimit){
                    this.drivetrain.setCurvatureDrive(AutoConstants.balanceCreepSpeed, 0, false);
                }
                else{
                    if (!atLimit){
                        atLimit = true;
                        limitPosition = this.drivetrain.getPose();
                        this.pidController.setSetpoint(0);
                        this.pidController.reset();
                    }
                    this.drivetrain.setCurvatureDrive(this.pidController.calculate(getRelativeDistance(limitPosition)), 0, false);
                }
                
            }
        }
        
        
    }

    private double getRelativeDistance(Pose2d position){
        return new Transform2d(position,this.drivetrain.getPose()).getTranslation().getX();
    }

    private boolean hasReachedChargeStation(){
        if (this.onChargeStation == false){
            if (Math.abs(this.robotPitchHandler.get()) > AutoConstants.onChargeStationAngle){
                this.onChargeStation = true;
                this.chargeCenterPosition = this.drivetrain.getPose().plus(new Transform2d(new Translation2d(-1 * AutoConstants.balanceCenterLimitFromInitialTip, 0.0), new Rotation2d()));
                return true;
            }
            return false;
            
        }
        return this.onChargeStation;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
