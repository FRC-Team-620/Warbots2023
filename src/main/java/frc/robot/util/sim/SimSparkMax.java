package frc.robot.util.sim;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotBase;

public class SimSparkMax extends CANSparkMax {
    private double lastSet = 0;

    public SimSparkMax(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public double get() {
        if (RobotBase.isSimulation()) {
            return this.lastSet;
        } else {
            return super.get();
        }

    }

    @Override
    public void set(double speed) {
        if (RobotBase.isSimulation()) {
            this.lastSet = speed;
        } else {
            super.set(speed);
        }

    }

}
