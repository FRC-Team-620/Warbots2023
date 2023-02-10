package com.revrobotics;

public class SparkMaxAnalogSensorSimWrapper {
	private SparkMaxAnalogSensor sensor;
	public SparkMaxAnalogSensorSimWrapper(SparkMaxAnalogSensor sensor) {
		this.sensor = sensor;
	}

	public void setVelocity(float velocity) {
		this.sensor.setSimVelocity(velocity);
	}
	public void setPosition(float position) {
		this.sensor.setSimPosition(position);
	}

}
