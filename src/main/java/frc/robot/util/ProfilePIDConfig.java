package frc.robot.util;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

class ProfilePIDConfig extends PIDConfig {
    public TrapezoidProfile.Constraints constraints;
    public double maxVelocity;
    public double maxAcceleration;

    public ProfilePIDConfig(double kP, double kI, double kD) {
        PIDConfig pid = new PIDConfig(kP,kI,kD);
        maxVelocity = 1;
        maxAcceleration = 0;
        constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }

    public ProfilePIDConfig(double kP, double kI, double kD, double TolError, double TolDerivError) {
        PIDConfig pid = new PIDConfig(kP,kI,kD, TolError, TolDerivError);
        maxVelocity = 1;
        maxAcceleration = 0;
        constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }

    public ProfilePIDConfig(double kP, double kI, double kD, double TolError, double TolDerivError, double VeloMax, double AccelMax) {
        PIDConfig pid = new PIDConfig(kP,kI,kD, TolError, TolDerivError);
        maxVelocity = VeloMax;
        maxAcceleration = AccelMax;
        constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }
}
