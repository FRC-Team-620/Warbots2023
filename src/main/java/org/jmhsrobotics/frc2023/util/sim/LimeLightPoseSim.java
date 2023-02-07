package org.jmhsrobotics.frc2023.util.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLightPoseSim {
    private LimeLightSim sim;
    private Pose2d target, pos;
    private double height, angle;

    public LimeLightPoseSim(LimeLightSim sim, Pose2d target, double height, double targetHeight, double angle) {
        this.sim = sim;
        this.target = target;
        this.height = targetHeight - height;
        this.angle = angle;

    }

    public void setPosition(Pose2d pos) {
        this.pos = pos;
    }

    public void update(double timeStep) {
        // target.g
        var yangle = Math.toDegrees(Math.atan2(height, target.getTranslation().getDistance(pos.getTranslation())) - Math.toRadians( this.angle));
        var deltap = target.minus(pos);
        var xangle = Math.toDegrees(Math.atan2(deltap.getY(), deltap.getX()));
        SmartDashboard.putNumber("PosSim/X angle",xangle);
        SmartDashboard.putNumber("PosSim/Y Angle",yangle);
        var xclamp = MathUtil.clamp(xangle, -29.8, 29.8);
        var yclamp = MathUtil.clamp(yangle, -24.85, 24.85);
        try {
            if(xclamp == xangle && yclamp == yangle){
                sim.setHasTarget(true);
                sim.setOffsetX(-xangle);
                sim.setOffsetY(yangle);
            }else{
                sim.setHasTarget(false);
                sim.setOffsetX(0);
                sim.setOffsetY(0);
            }
        } catch (Exception e) {
            e.printStackTrace();
            //TODO: handle exception
        }
        
       
//         Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
//      	Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
        //If angle is less than preset angle it needds to be neg

    }

}
