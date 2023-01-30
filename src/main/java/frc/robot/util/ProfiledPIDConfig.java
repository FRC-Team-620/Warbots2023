package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
public class ProfiledPIDConfig {
    public final double kp;
    public final double ki;
    public final double kd;
    public final double posTol;
    public final double velTol;
    public final Constraints constraints;  
    public ProfiledPIDConfig(double kp, double ki, double kd, Constraints constraints){
        this(kp,ki,kd,constraints, 0 ,0);

    }

    public ProfiledPIDConfig(double kp, double ki, double kd, Constraints constraints, double posTol, double velTol){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.posTol = posTol; 
        this.velTol = velTol; 
        this.constraints = constraints; 
    }   

    public static ProfiledPIDController getProfiledPIDController(ProfiledPIDConfig config) {
        ProfiledPIDController controller = new ProfiledPIDController(config.kp, config.ki , config.kd , config.constraints );
        controller.setTolerance(config.posTol, config.velTol);
        return controller;

    }
}