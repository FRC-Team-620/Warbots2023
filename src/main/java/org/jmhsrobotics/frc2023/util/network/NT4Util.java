package org.jmhsrobotics.frc2023.util.network;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The {@link NT4Util} helper class provides helper methods to put non-standard
 * datatypes into networktables via SmartDashboard.
 *
 */
public class NT4Util {

	/**
	 * Put a Pose3d array in the table.
	 *
	 * @param key
	 *            the key to be assigned to
	 * @param value
	 *            the value that will be assigned
	 * @return False if the table key already exists with a different type
	 */
	public static boolean putPose3d(String key, Pose3d... value) {
		double[] data = new double[value.length * 7];
		for (int i = 0; i < value.length; i++) {
			data[i * 7] = value[i].getX();
			data[i * 7 + 1] = value[i].getY();
			data[i * 7 + 2] = value[i].getZ();
			data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
			data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
			data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
			data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
		}
		return SmartDashboard.putNumberArray(key, data);
	}

	/**
	 * Put a Pose2d array in the table as a Pose3d (Assumes z=0)
	 *
	 * @param key
	 *            the key to be assigned to
	 * @param value
	 *            the value that will be assigned
	 * @return False if the table key already exists with a different type
	 */
	public static boolean putPose2d(String key, Pose2d... value) {
		Pose3d[] val3d = new Pose3d[value.length];
		for (int i = 0; i < value.length; i++) {
			val3d[i] = new Pose3d(value[i]);
		}
		return NT4Util.putPose3d(key, val3d);
	}

}
