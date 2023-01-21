package frc.robot.util.sim;
public interface IEncoderWrapper
{
    void setDistance(double distance);

    void setVelocity(double velocity);

    double getPosition();
}