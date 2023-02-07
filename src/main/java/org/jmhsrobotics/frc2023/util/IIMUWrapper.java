package org.jmhsrobotics.frc2023.util;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IIMUWrapper {

	public double getYaw();
	public double getPitch();
	public Rotation2d getRotation2d();
	public double getAngle();

}
