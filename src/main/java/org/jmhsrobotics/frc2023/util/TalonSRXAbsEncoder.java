package org.jmhsrobotics.frc2023.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonSRXAbsEncoder {

	/** Acts as a conduit to access the external encoder */
	TalonSRX talon;
	/** The CAN id (device number) of the TalonSRX */
	int id;

	public TalonSRXAbsEncoder(int CANId) {

		// initialize member variables
		this.id = CANId;
		this.talon = new TalonSRX(this.id);

		// configure for absolute encoder
		this.talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
	}

	public int getId() {
		return this.id;
	}

	public double getPosition() {
		return this.talon.getSelectedSensorPosition();
	}

	public double getVelocity() {
		return this.talon.getSelectedSensorVelocity();
	}
}
