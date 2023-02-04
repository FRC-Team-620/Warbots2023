package org.jmhsrobotics.frc2023.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
public class NavxIMU implements IIMUWrapper {

	private AHRS navx;

	public NavxIMU(SPI.Port port) { // SPI port
		this.navx = new AHRS(port);
	}

	public NavxIMU(I2C.Port port) {
		this.navx = new AHRS(port);
	}

	@Override
	public double getYaw() { // -180 180 in degrees
		return navx.getYaw();
	}

	@Override
	public double getPitch() {
		return navx.getPitch(); // -180 180 in degrees
	}

	@Override
	public Rotation2d getRotation2d() {
		return navx.getRotation2d();
	}

	@Override
	public double getAngle() {
		// TODO Auto-generated method stub
		return navx.getAngle();
	}

}
