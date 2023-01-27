package frc.robot.util.sim;

import java.util.function.DoubleConsumer;

import edu.wpi.first.hal.SimDouble;

public class BaseGyroWrapper implements IGyroWrapper
{
    private final DoubleConsumer mSetter;

    public BaseGyroWrapper(SimDouble simDouble)
    {
        this(simDouble::set);
    }

    public BaseGyroWrapper(DoubleConsumer setter)
    {
        mSetter = setter;
    }


    @Override
    public void setAngle(double angleDegrees)
    {
        mSetter.accept(angleDegrees);
    }
}