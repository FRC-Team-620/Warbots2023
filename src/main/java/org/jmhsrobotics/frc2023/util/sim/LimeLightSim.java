package org.jmhsrobotics.frc2023.util.sim;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSim {
    private NetworkTable ntable;
    private final NetworkTableEntry e_tv, e_tx, e_ty, e_ta, e_tl;
    private SimDevice m_simDevice;
    private SimBoolean s_tv;
    private SimDouble s_tx, s_ty, s_ta, s_tl;
    private static int index = 0;
    private final int id;

    public LimeLightSim() {
        this.id = index++;
        ntable = NetworkTableInstance.getDefault().getTable("limelight");
        e_tv = ntable.getEntry("tv");
        e_tx = ntable.getEntry("tx");
        e_ty = ntable.getEntry("ty");
        e_ta = ntable.getEntry("ta");
        e_tl = ntable.getEntry("tl");
        m_simDevice = SimDevice.create("LimeLight", this.id);
        s_tv = m_simDevice.createBoolean("Has Target", Direction.kOutput, false);
        s_tx = m_simDevice.createDouble("Offset X", Direction.kOutput, 0.0);
        s_ty = m_simDevice.createDouble("Offset Y", Direction.kOutput, 0.0);
        s_ta = m_simDevice.createDouble("Target Area", Direction.kOutput, 0.0);
        s_tl = m_simDevice.createDouble("Pipeline Latancy", Direction.kOutput, 0.0);
    }

    public void setHasTarget(boolean hastarget) {
        e_tv.setNumber(hastarget ? 1 : 0);
        s_tv.setValue(HALValue.makeBoolean(hastarget));
    }

    public void setOffsetY(double angle) {
        e_ty.setNumber(angle);
        s_ty.setValue(HALValue.makeDouble(angle));
    }

    public void setOffsetX(double angle) {
        // System.out.println(simd.enumerateValues().length);
        // System.out.println(simd.enumerateValues()[0]);

        e_tx.setNumber(angle);
        s_tx.setValue(HALValue.makeDouble(angle));
    }

    public void setArea(double area) {
        e_ta.setNumber(area);
        s_ta.setValue(HALValue.makeDouble(area));
    }

    public void setLatancy(double ms) {
        e_tl.setNumber(ms);
        s_tl.setValue(HALValue.makeDouble(ms));
    }

}
