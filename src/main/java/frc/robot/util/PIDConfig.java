package frc.robot.util;

public class PIDConfig {
    public static double P;
    public static double I;
    public static double D;
    public double TolError;
    public double TolDerivError;

    public PIDConfig() {
        P = 1;
        I = 1;
        D = 1;
        TolError = 1;
        TolError = 1;
    }

    public PIDConfig(double kP, double kI, double kD) {
        P = kP; 
        I = kI;
        D = kD;
    }
    
    public PIDConfig(double kP, double kI, double kD, double ToleranError, double ToleranDerivError) {
        P = kP; 
        I = kI;
        D = kD;
        TolError = ToleranError;
        TolDerivError = ToleranDerivError;
    }
}
