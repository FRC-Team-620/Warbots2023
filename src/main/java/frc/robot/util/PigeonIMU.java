package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

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
        double x = pigeon.getYaw();
        // if (pigeon.getYaw() > 180) {
        //     x = pigeon.getYaw() % 180 == 0 ? 180 : pigeon.getYaw() % 180;
        // } else if (pigeon.getYaw() < -180) {
        //     x = pigeon.getYaw() % 180 == 0 ? -180 : pigeon.getYaw() % 180;
        // }
        return x;
    }

    @Override
    public double getPitch() {
        double x = pigeon.getPitch();
        // if (pigeon.getPitch() > 180) {
        //     x = pigeon.getPitch() % 180 == 0 ? 180 : pigeon.getPitch() % 180;
        // } else if (pigeon.getPitch() < -180) {
        //     x = pigeon.getPitch() % 180 == 0 ? -180 : pigeon.getPitch() % 180;
        // }
        return x;
    }
    @Override
    public Rotation2d getRotation2d() {
        var rot2d = Rotation2d.fromDegrees(getYaw());
        return rot2d;
    }
    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return getYaw();
    }
}