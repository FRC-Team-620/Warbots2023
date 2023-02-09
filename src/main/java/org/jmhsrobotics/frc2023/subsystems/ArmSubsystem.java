package org.jmhsrobotics.frc2023.subsystems;

import org.jmhsrobotics.frc2023.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxAnalogSensorSimWrapper;

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
	SparkMaxAnalogSensor pitchEncoder = armPitch.getAnalog(Mode.kAbsolute);
	SparkMaxAnalogSensor extensionEncoder = armExtension.getAnalog(Mode.kAbsolute);
	public MechanismLigament2d m_elevator;
	public MechanismLigament2d m_wrist;
	public ProfiledPIDController profiledAnglePID;
	public ProfiledPIDController profiledExtensionPID;

	public ArmSubsystem() {
		// TODO: MAke sure to construct profiledExtensionPID and profiledExtensionPID!!!
		profiledAnglePID = new ProfiledPIDController(0.05, 0, 0.02, new Constraints(10, 10));
		profiledExtensionPID = new ProfiledPIDController(1, 0, 0, new Constraints(2, 1));
		// Create Mech2s display of Arm.
		// the mechanism root node
		Mechanism2d mech = new Mechanism2d(3, 3);
		MechanismRoot2d root = mech.getRoot("climber", 1, 0);
		var m_support = root.append(new MechanismLigament2d("support", 0.5, 90, 6, new Color8Bit(Color.kRed)));
		m_wrist = m_support.append(new MechanismLigament2d("wrist", 0.5, 0, 6, new Color8Bit(Color.kPurple)));
		m_elevator = m_wrist.append(new MechanismLigament2d("elevator", .5, 0));

		SmartDashboard.putData("arm_info", mech);
		SmartDashboard.putData("Wristpid", profiledAnglePID);
		SmartDashboard.putData("lengthpid", profiledExtensionPID);

	}

	@Override
	public void periodic() {
		armPitch.set(profiledAnglePID.calculate(pitchEncoder.getPosition()));
		armExtension.set(profiledExtensionPID.calculate(extensionEncoder.getPosition()));

		// Update Mech2d Display
		m_wrist.setAngle(pitchEncoder.getPosition() - 90);
		m_elevator.setLength(extensionEncoder.getPosition());

		SmartDashboard.putNumber("Wristpid/position", pitchEncoder.getPosition());
		SmartDashboard.putNumber("lengthpid/position", extensionEncoder.getPosition());
		SmartDashboard.putNumber("Wristpid/setpoint", profiledAnglePID.getSetpoint().position);
		SmartDashboard.putNumber("lengthpid/output", armExtension.get());
		SmartDashboard.putNumber("Wristpid/output", armPitch.get());
		SmartDashboard.putNumber("lengthpid/setpoint", profiledExtensionPID.getSetpoint().position);
	}

	// todo 1.make ports in own file

	// Sets the motor controlling arm height
	public void setPitchArmMotor(double targetAngleDeg) {

		profiledAnglePID.setGoal(new State(targetAngleDeg, 0));

	}

	// Sets the motor controlling arm length
	public void setExtensionArmMotor(double targetDistanceMeters) {
		profiledAnglePID.setGoal(new State(targetDistanceMeters, 0));

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
		double armLengthMeters = 1;
		double armMass = 2; // KG
		double moa = SingleJointedArmSim.estimateMOI(armLengthMeters, armMass);
		armsim = new SingleJointedArmSim(DCMotor.getNEO(1), 10, moa, armLengthMeters, Units.degreesToRadians(-360),
				Units.degreesToRadians(360), armMass, true);
		prismaticSim = new ElevatorSim(DCMotor.getNEO(1), 10, 1, Units.inchesToMeters(3), 0.3, 3, false);
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
		armsim.setInputVoltage(armvolts);
		prismaticSim.setInputVoltage(prismaticvolts);
		armsim.update(Constants.kSimUpdateTime);
		prismaticSim.update(Constants.kSimUpdateTime);
		pitchencsim.setPosition((float)Units.radiansToDegrees(armsim.getAngleRads()));
		pitchencsim.setVelocity((float)Units.radiansToDegrees(armsim.getVelocityRadPerSec())); // TODO should this be in rpm?

		extensionencsim.setPosition((float)prismaticSim.getPositionMeters());
		extensionencsim.setVelocity((float)prismaticSim.getVelocityMetersPerSecond()); // TODO should this be in rpm?

	}

}
