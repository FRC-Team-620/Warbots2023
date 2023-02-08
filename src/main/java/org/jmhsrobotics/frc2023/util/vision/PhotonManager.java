package org.jmhsrobotics.frc2023.util.vision;

import org.photonvision.PhotonCamera;

public class PhotonManager {
	public PhotonCamera mainCam;
	private static PhotonManager instance;
	private PhotonManager() {
		this.mainCam = new PhotonCamera("mainCam");
	}

	public static PhotonManager getInstance() {
		if (PhotonManager.instance == null) {
			PhotonManager.instance = new PhotonManager();
		}
		return PhotonManager.instance;
	}
}
