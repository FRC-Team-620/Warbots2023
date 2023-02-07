package org.jmhsrobotics.frc2023.util.sim;
public interface IEncoderWrapper
{
    void setDistance(double distance);

    void setVelocity(double velocity);

    double getPosition();
}