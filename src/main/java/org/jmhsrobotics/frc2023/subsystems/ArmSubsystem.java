package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.Constants.ArmConstants;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxAnalogSensorSimWrapper;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
	private TimeOfFlight laser = new TimeOfFlight(35);
	// Set the motors which power the basic functions of the arm
	private CANSparkMax pitchMotor = new CANSparkMax(Constants.driveports.getArmAngleCANId(), MotorType.kBrushless);
	private CANSparkMax telescopeMotor = new CANSparkMax(Constants.driveports.getArmExtensionCANId(),
			MotorType.kBrushless);
	private SparkMaxAnalogSensor pitchAbsoluteEncoder = pitchMotor.getAnalog(Mode.kAbsolute);
	private RelativeEncoder pitchEncoder = pitchMotor.getEncoder();
	private RelativeEncoder extensionEncoder = telescopeMotor.getEncoder();
	// private SparkMaxAnalogSensor extensionEncoder =
	// telescopeMotor.getAnalog(Mode.kAbsolute);
	private MechanismLigament2d m_elevator;
	private MechanismLigament2d m_wrist;
	private Constraints anglePPIDConstraints;
	private ProfiledPIDController profiledAnglePID;
	private Constraints extensionPPIDConstraints;
	private ProfiledPIDController profiledExtensionPID;
	private ArmFeedforward armfeedforward;
	private double openLoopExtensionSpeed;
	private double openLoopPitchSpeed;
	public static enum scoringType {
		CONE, CUBE;
	}
	public scoringType armScore = scoringType.CONE;

	public enum ControlMode {
		STOPPED, OPEN_LOOP, CLOSED_LOOP;
	}

	private ControlMode controlMode = ControlMode.CLOSED_LOOP;

	public ArmSubsystem() {

		armfeedforward = new ArmFeedforward(0, 0.46, 0.09); // Calculated from https://www.reca.lc/arm
		// TODO: MAke sure to construct profiledExtensionPID and profiledExtensionPID!!!

		// spotless:off
		anglePPIDConstraints = new Constraints(110, 200);
		profiledAnglePID = new ProfiledPIDController(
			0.10, 0.0, 0.0, anglePPIDConstraints
		);

		profiledAnglePID.setTolerance(1.5, 4);

		extensionPPIDConstraints = new Constraints(400, 700);
		profiledExtensionPID = new ProfiledPIDController(
			0.014, 0.0, 0.0, extensionPPIDConstraints
		);

		profiledExtensionPID.setTolerance(20, 60);

		laser.setRangingMode(RangingMode.Short, 25);
		// spotless:on
		SmartDashboard.putData("Wristpid", profiledAnglePID);
		SmartDashboard.putData("Lengthpid", profiledExtensionPID);
		// Create Mech2s display of Arm.
		// the mechanism root node
		init2d();
		// Set Current Limits
		pitchMotor.setSmartCurrentLimit(40);
		telescopeMotor.setSmartCurrentLimit(40);

		pitchEncoder.setPosition(0);

		telescopeMotor.setIdleMode(IdleMode.kBrake);
		// extensionEncoder.setPositionConversionFactor(ArmConstants.extensionMetersPerEncoderTick);
		extensionEncoder.setPosition(0);

		this.profiledAnglePID.setGoal(this.getArmPitch());
		this.profiledExtensionPID.setGoal(this.getArmLength());
	}

	public void init2d() {
		Mechanism2d mech = new Mechanism2d(3, 3);
		MechanismRoot2d root = mech.getRoot("climber", 1.5 + ArmConstants.armDistanceToCenterMeters, 0);
		var m_support = root.append(
				new MechanismLigament2d("support", ArmConstants.armHeightMeters, 90, 6, new Color8Bit(Color.kRed)));
		m_wrist = m_support.append(new MechanismLigament2d("wrist", ArmConstants.minExtensionLengthMillims, 0, 6,
				new Color8Bit(Color.kPurple)));
		m_elevator = m_wrist.append(new MechanismLigament2d("elevator", ArmConstants.minExtensionLengthMillims, 0, 6,
				new Color8Bit(Color.kGray)));
		SmartDashboard.putData("arm_info", mech);
	}

	@Override
	public void periodic() {
		laser.identifySensor();
		SmartDashboard.putNumber("ArmSubsystem/armPitch", this.getArmPitch());
		SmartDashboard.putNumber("ArmSubsystem/LaserDistance", laser.getRange());
		SmartDashboard.putString("arm/controlMode", getControlMode().toString());
		SmartDashboard.putNumber("ArmSubsystem/pitchMotorRelativeEncoder", pitchEncoder.getPosition() * -1.7379 + 30);
		SmartDashboard.putNumber("ArmSubsystem/PitchAbsoluteEncoderPosition", pitchAbsoluteEncoder.getPosition());
		// SmartDashboard.putNumber("ArmSubsystem/StringPotPosition",
		// extensionEncoder.getPosition());
		SmartDashboard.putNumber("ArmSubsystem/laser sensor", this.laser.getRange());
		SmartDashboard.putNumber("ArmSubsystem/RelativeEncoderExtension", telescopeMotor.getEncoder().getPosition());
		SmartDashboard.putNumber("ArmSubsystem/MaxLength", ArmConstants.maxExtensionLengthMillims);

		// Pitch logging
		SmartDashboard.putNumber("ArmSubsystem/Pitch/currentPitch", this.getArmPitch());
		SmartDashboard.putNumber("ArmSubsystem/Pitch/currentGoal", this.profiledAnglePID.getGoal().position);
		SmartDashboard.putNumber("ArmSubsystem/Pitch/currentSetpointPosition",
				this.profiledAnglePID.getSetpoint().position);
		SmartDashboard.putNumber("ArmSubsystem/Pitch/currentSetpointVelocity",
				this.profiledAnglePID.getSetpoint().velocity);

		// Extension logging
		SmartDashboard.putNumber("ArmSubsystem/Extension/currentExtension", this.getArmLength());
		SmartDashboard.putNumber("ArmSubsystem/Extension/currentGoal", this.profiledExtensionPID.getGoal().position);
		SmartDashboard.putNumber("ArmSubsystem/Extension/currentSetpointPosition",
				this.profiledExtensionPID.getSetpoint().position);
		SmartDashboard.putNumber("ArmSubsystem/Extension/currentSetpointVelocity",
				this.profiledExtensionPID.getSetpoint().velocity);

		// Update Mech2d Display
		m_wrist.setAngle(pitchAbsoluteEncoder.getPosition() - 90);
		m_elevator.setLength(extensionEncoder.getPosition());

		if (getControlMode() == ControlMode.STOPPED) {
			pitchMotor.set(0); // Keeps motor safety watchdog happy
			telescopeMotor.set(0);
			return;
		} else if (getControlMode() == ControlMode.OPEN_LOOP) {
			pitchMotor.set(openLoopPitchSpeed);
			telescopeMotor.set(openLoopExtensionSpeed);
			return;
		}

		// pitchMotor.set((armfeedforward.calculate(profiledAnglePID.getSetpoint().position,
		// profiledAnglePID.getSetpoint().velocity) +
		// profiledAnglePID.calculate(pitchEncoder.getPosition()))
		// / 12); // TODO fix janky volts hack

		pitchMotor.set(-profiledAnglePID.calculate(this.getArmPitch()));
		telescopeMotor.set(profiledExtensionPID.calculate(this.getArmLength()));
		// armExtension.set(profiledExtensionPID.getGoal().position);
		SmartDashboard.putNumber("ArmSubsystem/pitch_angle", this.getArmPitch());
		SmartDashboard.putNumber("ArmSubsystem/pitchPID/position_goal", this.profiledAnglePID.getGoal().position);
		SmartDashboard.putNumber("ArmSubsystem/pitchPID/position_setpoint",
				this.profiledAnglePID.getSetpoint().position);
		SmartDashboard.putNumber("ArmSubsystem/pitchPID/velocity_setpoint",
				this.profiledAnglePID.getSetpoint().velocity);
		// SmartDashboard.putNumber("lengthpid/spot",
		// profiledExtensionPID.calculate(extensionEncoder.getPosition()));
		// SmartDashboard.putNumber("Wristpid/position",
		// pitchAbsoluteEncoder.getPosition());
		// SmartDashboard.putNumber("lengthpid/position",
		// extensionEncoder.getPosition());
		// SmartDashboard.putNumber("Wristpid/setpoint",
		// profiledAnglePID.getSetpoint().position);
		// SmartDashboard.putNumber("lengthpid/output", telescopeMotor.get());
		// SmartDashboard.putNumber("Wristpid/output", pitchMotor.get());
		// SmartDashboard.putNumber("lengthpid/setpoint",
		// profiledExtensionPID.getSetpoint().position);
	}

	public void setControlMode(ControlMode mode) {
		this.controlMode = mode;
	}

	public void resetAnglePPIDToValue(double value) {
		this.profiledAnglePID.reset(value);
	}

	public void resetAnglePPIDToCurrent() {
		this.profiledAnglePID.reset(this.getArmPitch());
	}

	public void resetExtensionPPIDToCurrent() {
		this.profiledExtensionPID.reset(this.getArmLength());
	}

	public double getPitchRelativeEncoderPosition() {
		return this.pitchEncoder.getPosition();
	}

	public double getExtensionRelativeEncoderPosition() {
		return this.extensionEncoder.getPosition();
	}

	public State getPitchPPIDGoal() {
		return this.profiledAnglePID.getGoal();
	}

	public State getExtensionPPIDGoal() {
		return this.profiledExtensionPID.getGoal();
	}

	public void resetPitchEncoder() {
		this.pitchEncoder.setPosition(0.0);
	}

	// public void resetExtensionEncoder() {
	// this.extensionEncoder.setPosition(0.0);
	// }

	public double getMaxPitchPPIDVel() {
		return this.anglePPIDConstraints.maxVelocity;
	}

	public double getMaxExtensionPPIDVel() {
		return this.extensionPPIDConstraints.maxVelocity;
	}

	public void updatePitchGoal(double goal) {
		this.profiledAnglePID.setGoal(goal);
		this.controlMode = ControlMode.CLOSED_LOOP;
	}

	public void updateExtensionGoal(double goal) {
		this.profiledExtensionPID.setGoal(goal);
		this.controlMode = ControlMode.CLOSED_LOOP;
	}

	public double getArmPitch() {
		return ArmConstants.pitchDegreesPerEncoderTick * this.pitchEncoder.getPosition() + ArmConstants.stowedDegrees;
	}

	public static double pitchEncoderPositionFromDegrees(double angle) {
		return (angle - ArmConstants.stowedDegrees) / ArmConstants.pitchDegreesPerEncoderTick;
	}

	/**
	 * Raw duty cycle control of the arm motors. WARNING! does not respect soft
	 * limits!
	 *
	 * @param pitchSpeed
	 *            pitch motor duty cycle (-1,1)
	 * @param extentionSpeed
	 *            extention motor duty cycle (-1,1)
	 */
	public void setDutyCycle(double pitchSpeed, double extentionSpeed) {
		controlMode = ControlMode.OPEN_LOOP;
		openLoopExtensionSpeed = extentionSpeed;
		openLoopPitchSpeed = pitchSpeed;
	}

	// Sets the motor controlling arm height
	public void setPitch(double targetAngleDeg) {
		targetAngleDeg = MathUtil.clamp(targetAngleDeg, ArmConstants.minArmAngleDegrees,
				ArmConstants.maxArmAngleDegrees);
		profiledAnglePID.reset(this.getArmPitch());
		profiledAnglePID.setGoal(new State(targetAngleDeg, 0));
		SmartDashboard.putNumber("Wristpid/targetAngleDeg", targetAngleDeg);

		if (controlMode != ControlMode.CLOSED_LOOP) {
			profiledAnglePID.setGoal(new State(this.getArmPitch(), 0));
			profiledAnglePID.reset(this.getArmPitch());
		}
		controlMode = ControlMode.CLOSED_LOOP;
	}

	// Sets the motor controlling arm length
	public void setExtension(double targetDistanceMillims) { // TODO: Add back distance clamps for arm extention oops
		// targetDistanceMeters = MathUtil.clamp(targetDistanceMeters, 0,
		// ArmConstants.maxExtensionLengthMeters -
		// ArmConstants.minExtensionLengthMeters);.
		targetDistanceMillims = MathUtil.clamp(targetDistanceMillims, ArmConstants.minExtensionLengthMillims,
				ArmConstants.maxExtensionLengthMillims);
		profiledExtensionPID.reset(this.getArmLength());
		// TODO: not using meters using encoder counts so switch to meters
		profiledExtensionPID.setGoal(targetDistanceMillims); // TODO: Potential Bug because we reset the
																// goal we set here when switching into
																// closed loop control
		SmartDashboard.putNumber("ArmSubsystem/lengthpid/targetLength", profiledExtensionPID.getGoal().position);

		if (controlMode != ControlMode.CLOSED_LOOP) {
			profiledExtensionPID.setGoal(new State(this.getArmLength(), 0));
			profiledExtensionPID.reset(this.getArmLength());
		}
		SmartDashboard.putNumber("ArmSubsystem/lengthpid/targetLength__2", profiledExtensionPID.getGoal().position);
		controlMode = ControlMode.CLOSED_LOOP;
	}

	public void setScoringType() {
		if (armScore == scoringType.CONE) {
			armScore = scoringType.CUBE;
		} else {
			armScore = scoringType.CONE;
		}
	}

	/*
	 * Returns the angle of the arm in degrees. Zero degrees being parallel to the
	 * floor. CWW+
	 */
	public double getPitchMotor() {
		return pitchAbsoluteEncoder.getPosition();
	}

	/*
	 * Returns the arm extention length in millimeters
	 */
	public double getArmLength() {
		// return extensionEncoder.getPosition();
		return laser.getRange();
	}

	public boolean atPitchGoal() {
		return profiledAnglePID.atGoal();
	}

	public boolean atExtensionGoal() {
		return profiledExtensionPID.atGoal();
	}

	public ControlMode getControlMode() {
		return controlMode;
	}

	public double getPitchPosition() {
		return this.pitchAbsoluteEncoder.getPosition();
	}

	public void stop() {
		controlMode = ControlMode.STOPPED;

	}

	/**
	 * Simulation Code
	 */
	private boolean simInit = false;
	SingleJointedArmSim armsim;
	ElevatorSim prismaticSim;
	private SparkMaxAnalogSensorSimWrapper pitchencsim;
	private SparkMaxAnalogSensorSimWrapper extensionencsim;

	private void initSim() {
		// Constants.ArmConstants.minExtensionLengthMeters
		double moi = SingleJointedArmSim.estimateMOI(ArmConstants.minExtensionLengthMillims, ArmConstants.armMasskg);
		armsim = new SingleJointedArmSim(DCMotor.getNEO(1), ArmConstants.armPitchGearRatio, moi,
				ArmConstants.maxExtensionLengthMillims, Units.degreesToRadians(ArmConstants.minArmAngleDegrees),
				Units.degreesToRadians(ArmConstants.maxArmAngleDegrees), true);
		prismaticSim = new ElevatorSim(DCMotor.getNeo550(1), 10, 10, Units.inchesToMeters(3),
				ArmConstants.minExtensionLengthMillims, ArmConstants.maxExtensionLengthMillims, false);
		pitchencsim = new SparkMaxAnalogSensorSimWrapper(pitchAbsoluteEncoder);
		// extensionencsim = new SparkMaxAnalogSensorSimWrapper(extensionEncoder);
	}

	@Override
	public void simulationPeriodic() {
		if (!simInit) {
			initSim();
			simInit = true;
		}
		var armvolts = pitchMotor.get() * RobotController.getInputVoltage();
		var prismaticvolts = telescopeMotor.get() * RobotController.getInputVoltage();
		if (RobotState.isDisabled()) {
			armvolts = 0;
			prismaticvolts = 0;
		}
		armsim.setInputVoltage(MathUtil.clamp(armvolts, -12, 12));
		prismaticSim.setInputVoltage(MathUtil.clamp(prismaticvolts, -12, 12));
		armsim.update(Constants.kSimUpdateTime);
		prismaticSim.update(Constants.kSimUpdateTime);
		pitchencsim.setPosition((float) Units.radiansToDegrees(armsim.getAngleRads()));
		pitchencsim.setVelocity((float) Units.radiansToDegrees(armsim.getVelocityRadPerSec())); // TODO should this be
																								// in rpm?

		extensionencsim
				.setPosition((float) (prismaticSim.getPositionMeters() - ArmConstants.minExtensionLengthMillims));
		extensionencsim.setVelocity((float) prismaticSim.getVelocityMetersPerSecond()); // TODO should this be in rpm?

	}

}
