package org.jmhsrobotics.frc2023.util.vision;

import org.jmhsrobotics.frc2023.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonManager {
	public PhotonCamera mainCam;
	private static PhotonManager instance;
	private static PhotonTrackedTarget target = new PhotonTrackedTarget();
	private PhotonManager() {
		this.mainCam = new PhotonCamera("mainCam");
		mainCam.setDriverMode(true);
		mainCam.setLED(VisionLEDMode.kOn);
	}

	public static PhotonManager getInstance() {
		if (PhotonManager.instance == null) {
			PhotonManager.instance = new PhotonManager();
		}
		return PhotonManager.instance;
	}

	// fudgeAngle gives you how many degrees you have to turn the robot to align
	// with the target based on the data from the limelight
	public static double fudgeAngle() {
		double angle = target.getYaw();
		// the angle we should turn the robot
		double fudgeAngle = Math.atan((Constants.fudgeAngleConstant.distance * Math.tan(angle))
				/ (Constants.fudgeAngleConstant.limelightHeight - Constants.fudgeAngleConstant.targetHeight));
		return fudgeAngle;
	}
}
