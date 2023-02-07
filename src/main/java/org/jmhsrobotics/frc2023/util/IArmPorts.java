package org.jmhsrobotics.frc2023.util;

import org.jmhsrobotics.frc2023.Constants;

public interface IArmPorts {
	public int getArmVerticalCANId();
	public int getArmHorizontalCanId();


	public static IArmPorts getArmPorts(Constants.RobotType type) {
		switch (type) {
			case SUSAN :
				return new SusanArmPorts();
			case BABY_BOT :
				return new BabybotArmPorts();
			case BOT_2020 :
				return new Bot2020DrivePorts();
			default :
				System.err.println("WARNING!: DEFAULTED TO BABYBOT");
				return new BabybotDrivePorts(); // TODO: Default to babybot
		}
	}
}
