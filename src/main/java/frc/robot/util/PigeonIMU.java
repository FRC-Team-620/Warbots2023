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
        return pigeon.getYaw();
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch();
    }
    @Override
    public Rotation2d getRotation2d() {
        var rot2d = new Rotation2d(getYaw()); //TODO: Validate this is works with range -180 to 180;
        return rot2d;
    }
    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return pigeon.getYaw();
    }
}