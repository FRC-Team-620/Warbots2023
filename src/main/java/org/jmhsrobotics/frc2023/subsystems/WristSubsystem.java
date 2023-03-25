package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.Constants.ControlMode;
import org.jmhsrobotics.frc2023.Constants.WristConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

	// wrist hardware
	private CANSparkMax wristMotor = new CANSparkMax(Constants.driveports.getWristCANId(), MotorType.kBrushless);
	private TalonSRX wristAbsoluteEncoderConduit = new TalonSRX(Constants.driveports.getWristAbsoluteEncoderCANId());

	private TrapezoidProfile.Constraints wristPPIDConstraints;
	private ProfiledPIDController wristPPID;

	private double wristAbsolutePosition;

	private double openLoopPitchSpeed = 0.0;
	private ControlMode controlMode = ControlMode.CLOSED_LOOP;

	public WristSubsystem() {

		this.wristMotor.setSmartCurrentLimit(20);
		this.wristMotor.setIdleMode(IdleMode.kBrake);
		// this.wristMotor.setInverted(true);

		this.wristAbsoluteEncoderConduit.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
		this.updateStoredWristPosition();

		// all in degrees
		this.wristPPIDConstraints = new TrapezoidProfile.Constraints(110, 200);
		// spotless:off
        this.wristPPID = new ProfiledPIDController(
            0.0/*0.1*/, 0, 0, 
            this.wristPPIDConstraints
        );
        // spotless:on
		this.wristPPID.setTolerance(1, 20);

		this.wristPPID.setGoal(this.getWristPitch());
	}

	@Override
	public void periodic() {

		SmartDashboard.putNumber("WristSubsystem/wrist/speed", this.getWristSpeed());
		SmartDashboard.putNumber("WristSubsystem/wrist/absolute position", this.getWristPosition());
		SmartDashboard.putNumber("WristSubsystem/wrist/pitch degrees", this.getWristPitch());
		SmartDashboard.putNumber("WristSubsystem/wrist/PPID/goal position", this.getWristPPIDGoal().position);
		SmartDashboard.putNumber("WristSubsystem/wrist/PPID/setpoint position", this.wristPPID.getSetpoint().position);
		SmartDashboard.putNumber("WristSubsystem/wrist/PPID/setpoint velocity", this.wristPPID.getSetpoint().velocity);

		// IMPORTANT
		// updates the stored absolute encoder value for this tick
		// (so that the encoder is not polled more than once per tick)
		this.updateStoredWristPosition(); // a private method

		// handling different control modes
		switch (this.getControlMode()) {
			case STOPPED :
				this.openLoopPitchSpeed = 0.0;
				this.stopWristMotor();
				return; // STOPPED STATE STOPS HERE
			case OPEN_LOOP : // manual/duty cycle control
				this.setWristMotor(this.openLoopPitchSpeed);
				return; // OPEN LOOP STOPS HERE
			case CLOSED_LOOP :
				break; // continue with periodic
		}

		// CLOSED LOOP

		// make sure there's nothing stored between two bouts of open loop control
		this.openLoopPitchSpeed = 0.0;

		// PPID control
		this.setWristMotor(this.wristPPID.calculate(this.getWristPitch()));
	}

	// ****** WRIST MOTOR ******

	public void setWristMotor(double speed) {
		this.wristMotor.set(speed);
	}

	public double getWristSpeed() {
		return this.wristMotor.get();
	}

	public void stopWristMotor() {
		this.wristMotor.set(0.0);
	}

	public double getWristPosition() {
		return this.wristAbsolutePosition;
	}

	public double getWristPitch() {
		return (this.getWristPosition() - WristConstants.encoderTicksAtZeroDegrees)
				* WristConstants.degreesPerEncoderTick;
	}

	private void updateStoredWristPosition() {
		this.wristAbsolutePosition = this.wristAbsoluteEncoderConduit.getSelectedSensorPosition();
	}

	// ****** WRIST CONTROL ******

	public void setControlMode(ControlMode controlMode) {
		this.controlMode = controlMode;
	}

	public ControlMode getControlMode() {
		return this.controlMode;
	}

	public void stop() {
		this.setControlMode(ControlMode.STOPPED);
	}

	public void setDutyCycle(double pitchSpeed) {
		this.controlMode = ControlMode.OPEN_LOOP;
		this.openLoopPitchSpeed = MathUtil.clamp(pitchSpeed, -1.0, 1.0);
	}

	public void setPitch(double pitchGoal) {

		if (this.getControlMode() != ControlMode.CLOSED_LOOP)
			this.resetWristPPIDToCurrent();

		pitchGoal = MathUtil.clamp(pitchGoal, WristConstants.minPitchDegrees, WristConstants.maxPitchDegrees);
		this.wristPPID.setGoal(pitchGoal);

		this.controlMode = ControlMode.CLOSED_LOOP;
	}

	public void resetWristPPIDToCurrent() {
		this.wristPPID.reset(this.getWristPitch());
	}

	public TrapezoidProfile.Constraints getWristPPIDConstraints() {
		return this.wristPPIDConstraints;
	}

	public TrapezoidProfile.State getWristPPIDGoal() {
		return this.wristPPID.getGoal();
	}
}
