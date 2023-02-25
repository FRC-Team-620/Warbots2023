package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxAnalogSensorSimWrapper;

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
	// Set the motors which power the basic functions of the arm
	// private Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 42);
	private CANSparkMax pitchMotor = new CANSparkMax(Constants.driveports.getArmAngleCANId(), MotorType.kBrushless);
	private CANSparkMax telescopeMotor = new CANSparkMax(Constants.driveports.getArmExtensionCANId(),
			MotorType.kBrushless);
	private SparkMaxAnalogSensor pitchEncoder = pitchMotor.getAnalog(Mode.kAbsolute);
	private SparkMaxAnalogSensor extensionEncoder = telescopeMotor.getAnalog(Mode.kAbsolute);
	private MechanismLigament2d m_elevator;
	private MechanismLigament2d m_wrist;
	private ProfiledPIDController profiledAnglePID;
	private ProfiledPIDController profiledExtensionPID;
	private ArmFeedforward armfeedforward;
	private double openLoopExtensionSpeed;
	private double openLoopPitchSpeed;

	private enum ControlMode {
		STOPPED, OPEN_LOOP, CLOSED_LOOP;
	}

	private ControlMode controlMode = ControlMode.CLOSED_LOOP;

	public ArmSubsystem() {

		armfeedforward = new ArmFeedforward(0, 0.46, 0.09); // Calculated from https://www.reca.lc/arm
		// TODO: MAke sure to construct profiledExtensionPID and profiledExtensionPID!!!
		profiledAnglePID = new ProfiledPIDController(0.05, 0.01, 0.02, new Constraints(90, 360));
		profiledExtensionPID = new ProfiledPIDController(25, 0, 1, new Constraints(1, 0));
		SmartDashboard.putData("Wristpid", profiledAnglePID);
		SmartDashboard.putData("lengthpid", profiledExtensionPID);
		// Create Mech2s display of Arm.
		// the mechanism root node
		init2d();
		// Set Current Limits
		pitchMotor.setSmartCurrentLimit(40);
		telescopeMotor.setSmartCurrentLimit(40);
		controlMode = ControlMode.STOPPED;
		// System.out.println("HELLLLLLLOOOOO");
	}

	public void init2d() {
		Mechanism2d mech = new Mechanism2d(3, 3);
		MechanismRoot2d root = mech.getRoot("climber", 1.5 + ArmConstants.armDistanceToCenterMeters, 0);
		var m_support = root.append(
				new MechanismLigament2d("support", ArmConstants.armHeightMeters, 90, 6, new Color8Bit(Color.kRed)));
		m_wrist = m_support.append(new MechanismLigament2d("wrist", ArmConstants.minExtensionLengthMeters, 0, 6,
				new Color8Bit(Color.kPurple)));
		m_elevator = m_wrist.append(new MechanismLigament2d("elevator", ArmConstants.minExtensionLengthMeters, 0, 6,
				new Color8Bit(Color.kGray)));
		SmartDashboard.putData("arm_info", mech);
	}

	@Override
	public void periodic() {
		// System.out.println("PERIODDIC");

		SmartDashboard.putNumber("ArmSubsystem/PitchAbsoluteEncoderPosition", pitchEncoder.getPosition());
		if (getControlMode() == ControlMode.STOPPED) {
			pitchMotor.set(0); // Keeps motor safety watchdog happy
			telescopeMotor.set(0);
			return;
		} else if (getControlMode() == ControlMode.OPEN_LOOP) {
			pitchMotor.set(openLoopPitchSpeed);
			telescopeMotor.set(openLoopExtensionSpeed);
			return;
		}

		pitchMotor.set((armfeedforward.calculate(profiledAnglePID.getSetpoint().position,
				profiledAnglePID.getSetpoint().velocity) + profiledAnglePID.calculate(pitchEncoder.getPosition()))
				/ 12); // TODO fix janky volts hack

		telescopeMotor.set(profiledExtensionPID.calculate(extensionEncoder.getPosition()) / 12.0);
		// armExtension.set(profiledExtensionPID.getGoal().position);
		// Update Mech2d Display
		m_wrist.setAngle(pitchEncoder.getPosition() - 90);
		m_elevator.setLength(extensionEncoder.getPosition());
		SmartDashboard.putNumber("lengthpid/spot", profiledExtensionPID.calculate(extensionEncoder.getPosition()));
		SmartDashboard.putNumber("Wristpid/position", pitchEncoder.getPosition());
		SmartDashboard.putNumber("lengthpid/position", extensionEncoder.getPosition());
		SmartDashboard.putNumber("Wristpid/setpoint", profiledAnglePID.getSetpoint().position);
		SmartDashboard.putNumber("lengthpid/output", telescopeMotor.get());
		SmartDashboard.putNumber("Wristpid/output", pitchMotor.get());
		SmartDashboard.putNumber("lengthpid/setpoint", profiledExtensionPID.getSetpoint().position);
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
		profiledAnglePID.setGoal(new State(targetAngleDeg, 0));
		SmartDashboard.putNumber("Wristpid/targetAngleDeg", targetAngleDeg);

		if (controlMode != ControlMode.CLOSED_LOOP) {
			profiledExtensionPID.setGoal(new State(extensionEncoder.getPosition(), 0));
			profiledExtensionPID.reset(new State(extensionEncoder.getPosition(), 0));
		}
		controlMode = ControlMode.CLOSED_LOOP;
	}

	// Sets the motor controlling arm length
	public void setExtension(double targetDistanceMeters) { // TODO: Add back distance clamps for arm extention oops
		// targetDistanceMeters = MathUtil.clamp(targetDistanceMeters, 0,
		// ArmConstants.maxExtensionLengthMeters -
		// ArmConstants.minExtensionLengthMeters);
		profiledExtensionPID.setGoal(new State(targetDistanceMeters, 0)); // TODO: Potential Bug because we reset the
																			// goal we set here when switching into
																			// closed loop control
		SmartDashboard.putNumber("lengthpid/targetLength", targetDistanceMeters);

		if (controlMode != ControlMode.CLOSED_LOOP) {
			profiledAnglePID.setGoal(new State(pitchEncoder.getPosition(), 0));
			profiledAnglePID.reset(new State(pitchEncoder.getPosition(), 0));
		}
		controlMode = ControlMode.CLOSED_LOOP;
	}

	/*
	 * Returns the angle of the arm in degrees. Zero degrees being parallel to the
	 * floor. CWW+
	 */
	public double getPitchMotor() {
		return pitchEncoder.getPosition();
	}

	/*
	 * Returns the arm extention length in meters
	 */
	public double getArmLength() {
		return extensionEncoder.getPosition();
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
		double moi = SingleJointedArmSim.estimateMOI(ArmConstants.minExtensionLengthMeters, ArmConstants.armMasskg);
		armsim = new SingleJointedArmSim(DCMotor.getNEO(1), ArmConstants.armPitchGearRatio, moi,
				ArmConstants.maxExtensionLengthMeters, Units.degreesToRadians(ArmConstants.minArmAngleDegrees),
				Units.degreesToRadians(ArmConstants.maxArmAngleDegrees), true);
		prismaticSim = new ElevatorSim(DCMotor.getNeo550(1), 10, 10, Units.inchesToMeters(3),
				ArmConstants.minExtensionLengthMeters, ArmConstants.maxExtensionLengthMeters, false);
		pitchencsim = new SparkMaxAnalogSensorSimWrapper(pitchEncoder);
		extensionencsim = new SparkMaxAnalogSensorSimWrapper(extensionEncoder);
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

		extensionencsim.setPosition((float) (prismaticSim.getPositionMeters() - ArmConstants.minExtensionLengthMeters));
		extensionencsim.setVelocity((float) prismaticSim.getVelocityMetersPerSecond()); // TODO should this be in rpm?

	}

}
