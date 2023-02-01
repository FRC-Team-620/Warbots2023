package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonIMU implements IIMUWrapper{
    private Pigeon2 pigeon;
    public PigeonIMU(int canId){
        pigeon = new Pigeon2(canId);
    }
    public PigeonIMU(int canId, Pigeon2Configuration config){
        this(canId);
        pigeon.configAllSettings(config);
    }

    @Override
    public double getYaw() {    
        // double x = pigeon.getYaw();
        // x = -MathUtil.inputModulus(x, -180, 180);
        // if (pigeon.getYaw() > 180) {
        //     x = pigeon.getYaw() % 180 == 0 ? 180 : pigeon.getYaw() % 180;
        // } else if (pigeon.getYaw() < -180) {
        //     x = pigeon.getYaw() % 180 == 0 ? -180 : pigeon.getYaw() % 180;
        // }
        return PigeonIMU.pigeonConstrain180(pigeon.getYaw());
    }

    @Override
    public double getPitch() {
        // double x = pigeon.getPitch();
        // x = -MathUtil.inputModulus(x, -180, 180);
        // if (pigeon.getPitch() > 180) {
        //     x = pigeon.getPitch() % 180 == 0 ? 180 : pigeon.getPitch() % 180;
        // } else if (pigeon.getPitch() < -180) {
        //     x = pigeon.getPitch() % 180 == 0 ? -180 : pigeon.getPitch() % 180;
        // }
        return PigeonIMU.pigeonConstrain180(pigeon.getPitch());
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return getYaw();
    }

    private static double pigeonConstrain180(double angle) {
        return -MathUtil.inputModulus(angle, -180, 180);
    }
}