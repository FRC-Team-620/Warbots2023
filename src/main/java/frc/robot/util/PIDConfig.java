package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class PIDConfig {
    public final double kp; 
    public final double ki;
    public final double kd;
    public final double posTol;
    public final double velTol; 
    public PIDConfig(double kp, double ki, double kd){
        this(kp,ki,kd, 0 ,0);
    }

    public PIDConfig(double kp, double ki, double kd, double posTol, double velTol){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.posTol = posTol; 
        this.velTol = velTol; 
    }

    public static PIDController getPIDController(PIDConfig config){
        PIDController controller = new PIDController(config.kp, config.ki, config.kd);
        controller.setTolerance(config.posTol, config.velTol);
         return controller; 

    }
}