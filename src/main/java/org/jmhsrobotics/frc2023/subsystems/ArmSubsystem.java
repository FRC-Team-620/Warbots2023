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
	private CANSparkMax armPitch = new CANSparkMax(5, MotorType.kBrushless);
	private CANSparkMax armExtension = new CANSparkMax(6, MotorType.kBrushless);
	private SparkMaxAnalogSensor pitchEncoder = armPitch.getAnalog(Mode.kAbsolute);
	private SparkMaxAnalogSensor extensionEncoder = armExtension.getAnalog(Mode.kAbsolute);
	private MechanismLigament2d m_elevator;
	private MechanismLigament2d m_wrist;
	private ProfiledPIDController profiledAnglePID;
	private ProfiledPIDController profiledExtensionPID;
	private ArmFeedforward armfeedforward;
	private boolean isStopped = false;

	public ArmSubsystem() {

		armfeedforward = new ArmFeedforward(0, 0.46, 0.09); // Calculated from https://www.reca.lc/arm
		// TODO: MAke sure to construct profiledExtensionPID and profiledExtensionPID!!!
		profiledAnglePID = new ProfiledPIDController(0.05, 0.01, 0.02, new Constraints(90, 360));
		profiledExtensionPID = new ProfiledPIDController(25, 0, 1, new Constraints(1, 0));
		// Create Mech2s display of Arm.
		// the mechanism root node
		Mechanism2d mech = new Mechanism2d(3, 3);
		MechanismRoot2d root = mech.getRoot("climber", 1, 0);
		var m_support = root.append(
				new MechanismLigament2d("support", ArmConstants.armHeightMeters, 90, 6, new Color8Bit(Color.kRed)));
		m_wrist = m_support
				.append(new MechanismLigament2d("wrist", Units.inchesToMeters(25), 0, 6, new Color8Bit(Color.kPurple)));
		m_elevator = m_wrist.append(new MechanismLigament2d("elevator", ArmConstants.minExtensionLengthMeters, 0, 6,
				new Color8Bit(Color.kGray)));
		armPitch.setSmartCurrentLimit(40);
		armExtension.setSmartCurrentLimit(15);
		SmartDashboard.putData("arm_info", mech);
		SmartDashboard.putData("Wristpid", profiledAnglePID);
		SmartDashboard.putData("lengthpid", profiledExtensionPID);

	}

	@Override
	public void periodic() {
		if (isStopped) {
			return;
		}

		armPitch.set((armfeedforward.calculate(profiledAnglePID.getSetpoint().position,
				profiledAnglePID.getSetpoint().velocity) + profiledAnglePID.calculate(pitchEncoder.getPosition()))
				/ 12); // TODO fix janky volts hack

		armExtension.set(profiledExtensionPID.calculate(extensionEncoder.getPosition()) / 12.0);
		// armExtension.set(profiledExtensionPID.getGoal().position);
		// Update Mech2d Display
		m_wrist.setAngle(pitchEncoder.getPosition() - 90);
		m_elevator.setLength(extensionEncoder.getPosition());

		SmartDashboard.putNumber("lengthpid/spot", profiledExtensionPID.calculate(extensionEncoder.getPosition()));
		SmartDashboard.putNumber("Wristpid/position", pitchEncoder.getPosition());
		SmartDashboard.putNumber("lengthpid/position", extensionEncoder.getPosition());
		SmartDashboard.putNumber("Wristpid/setpoint", profiledAnglePID.getSetpoint().position);
		SmartDashboard.putNumber("lengthpid/output", armExtension.get());
		SmartDashboard.putNumber("Wristpid/output", armPitch.get());
		SmartDashboard.putNumber("lengthpid/setpoint", profiledExtensionPID.getSetpoint().position);

	}

	// todo 1.make ports in own file

	// Sets the motor controlling arm height
	public void setArmPitch(double targetAngleDeg) {
		targetAngleDeg = MathUtil.clamp(targetAngleDeg, ArmConstants.minArmAngleDegrees,
				ArmConstants.maxArmAngleDegrees);
		profiledAnglePID.setGoal(new State(targetAngleDeg, 0));
		SmartDashboard.putNumber("Wristpid/targetAngleDeg", targetAngleDeg);

		if (isStopped) {
			profiledExtensionPID.setGoal(new State(extensionEncoder.getPosition(), 0));
			profiledExtensionPID.reset(new State(extensionEncoder.getPosition(), 0));
		}

		isStopped = false;

	}

	// Sets the motor controlling arm length
	public void setArmExtension(double targetDistanceMeters) {
		// targetDistanceMeters = MathUtil.clamp(targetDistanceMeters, 0,
		// ArmConstants.maxExtensionLengthMeters -
		// ArmConstants.minExtensionLengthMeters);
		profiledExtensionPID.setGoal(new State(targetDistanceMeters, 0));
		SmartDashboard.putNumber("lengthpid/targetLength", targetDistanceMeters);

		if (isStopped) {
			profiledAnglePID.setGoal(new State(pitchEncoder.getPosition(), 0));
			profiledAnglePID.reset(new State(pitchEncoder.getPosition(), 0));
		}
		isStopped = false;

	}

	/*
	 * Returns the angle of the arm in degrees. Zero degrees being parallel to the
	 * floor. CWW+
	 */
	public double getArmPitch() {
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

	public boolean isStopped() {
		return isStopped;
	}
	public void stop() {
		isStopped = true;
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
				ArmConstants.armLengthMeters, Units.degreesToRadians(ArmConstants.minArmAngleDegrees),
				Units.degreesToRadians(ArmConstants.maxArmAngleDegrees), ArmConstants.armMasskg, true);
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
		var armvolts = armPitch.get() * RobotController.getInputVoltage();
		var prismaticvolts = armExtension.get() * RobotController.getInputVoltage();
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
