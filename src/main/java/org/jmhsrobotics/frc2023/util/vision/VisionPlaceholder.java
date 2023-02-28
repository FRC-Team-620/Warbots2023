package org.jmhsrobotics.frc2023.util.vision;

import java.io.IOException;
import java.util.ArrayList;

import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.util.network.NT4Util;
import org.jmhsrobotics.frc2023.util.sim.SimPipeLineVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPlaceholder extends SubsystemBase {

	public static final Field2d field = new Field2d();
	private SimPipeLineVisionSystem simPhoton;
	private Drivetrain drivetrain;
	private Transform3d camPos = new Transform3d(new Translation3d(0, 0, 1), // TODO: Make a constant
			new Rotation3d(Units.degreesToRadians(180), 0, 0));

	/*
	 * This is currently a placeholder class to hold all the vision simulation stuff
	 * as well as the field2d odometry display
	 *
	 */
	public VisionPlaceholder(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		loadAprilTags();
		// Port forward PhotonVision over usb Accessable at roborio.local:5800 (Or what
		// ever the rio's ip is)
		PortForwarder.add(5800, "photonvision.local", 5800); // Web interface
		PortForwarder.add(1181, "photonvision.local", 1181); // Cam1 Mpeg stream
		PortForwarder.add(1182, "photonvision.local", 1182); // Cam2 Mpeg stream
		PortForwarder.add(1183, "photonvision.local", 1183); // Cam3 Mpeg stream
		SmartDashboard.putData(VisionPlaceholder.field);
	}

	@Override
	public void periodic() {

	}

	private void updateField() {
		NT4Util.putPose3d("camerapos", new Pose3d(drivetrain.getPose()).plus(camPos));
		// TODO: need to move the simulation calculations for target positions into the
		// simulation periodic not periodic
		field.setRobotPose(drivetrain.getPose()); // Updates robot's position on field2d
		this.simPhoton.processFrame(drivetrain.getPose());

		var result = PhotonManager.getInstance().mainCam.getLatestResult(); // TODO: handle switch between sim and real
																			// world.
		var ids = new ArrayList<Double>();
		var tagpos = new ArrayList<Pose2d>();
		ArrayList<Pose3d> tag3d = new ArrayList<Pose3d>();
		if (result.hasTargets()) {
			for (var target : result.getTargets()) {
				ids.add(target.getFiducialId() + 0.0);
				Pose3d tag = new Pose3d(drivetrain.getPose()).plus(camPos).plus((target.getBestCameraToTarget()));
				tagpos.add(tag.toPose2d());
				tag3d.add(tag);
			}
		}
		SmartDashboard.putNumberArray("vision/tagIds", ids.toArray(new Double[ids.size()]));
		NT4Util.putPose3d("vision/aprilTags", tag3d.toArray(new Pose3d[tag3d.size()]));
		field.getObject("Tags").setPoses(tagpos); // Display simulated tag positions
	}

	private void loadAprilTags() {
		try {
			AprilTagFieldLayout layout = new AprilTagFieldLayout(
					Filesystem.getDeployDirectory().getAbsolutePath() + "/2023-chargedup.json");

			SimVisionTarget target = new SimVisionTarget(new Pose3d(6, 6, 1.3, new Rotation3d(25, 25, 25)), 0.2, 0.2,
					0);

			simPhoton = new SimPipeLineVisionSystem("mainCam", 90, camPos, 20, 640, 480, 10);
			simPhoton.getPipeline(0).addVisionTargets(layout);
			simPhoton.getPipeline(simPhoton.addPipeline()).addSimVisionTarget(target);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	@Override
	public void simulationPeriodic() {
		updateField();
	}

}
