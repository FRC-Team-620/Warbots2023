package org.jmhsrobotics.frc2023.util.sim;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class SimPipeLineVisionSystem{
    ArrayList<SimVisionSystem> systems = new ArrayList<>();
    private final String camName;
    private final PhotonCamera cam;
    private final double camDiagFOVDegrees;
    private Transform3d robotToCamera;
    private final double maxLEDRangeMeters, minTargetArea;
    private final int  cameraResWidth, cameraResHeight;
    private int lastPipeline = 0;
    public SimPipeLineVisionSystem (String camName, double camDiagFOVDegrees, Transform3d robotToCamera, double maxLEDRangeMeters, int cameraResWidth, int cameraResHeight, double minTargetArea){
        this.camName = camName;
        this.camDiagFOVDegrees = camDiagFOVDegrees;
        this.robotToCamera = robotToCamera;
        this.maxLEDRangeMeters = maxLEDRangeMeters;
        this.cameraResHeight = cameraResHeight;
        this.cameraResWidth = cameraResWidth;
        this.minTargetArea = minTargetArea;
        this.cam = new PhotonCamera(camName);
        this.addPipeline();
    }
    public int addPipeline(){
        
         systems.add(new SimVisionSystem(camName,camDiagFOVDegrees,robotToCamera,maxLEDRangeMeters,cameraResWidth,cameraResHeight,minTargetArea));
         return systems.size() -1;
    }

    public SimVisionSystem getPipeline(int index){
        try{
            return systems.get(index);
        }catch(ArrayIndexOutOfBoundsException e){
            return null;
        }
    }

    public void processFrame(Pose2d  robotPoseMeters){
        this.processFrame(new Pose3d(robotPoseMeters));

    }

    public void processFrame(Pose3d  robotPoseMeters){
        int index = this.cam.getPipelineIndex();
        SimVisionSystem pipeline = this.getPipeline(index);
        if(pipeline != null){
            pipeline.processFrame( robotPoseMeters);
        }else{
            this.systems.get(lastPipeline).processFrame(robotPoseMeters);
        }

    }
    public void moveCamera(Transform3d newRobotToCamera) {
        this.robotToCamera = newRobotToCamera;
        for(SimVisionSystem sys: this.systems){
            sys.moveCamera(newRobotToCamera);
        }
    }
    
}
