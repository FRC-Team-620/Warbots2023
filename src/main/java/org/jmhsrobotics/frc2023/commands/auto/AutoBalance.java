package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.math.MathUtil;

//import org.apache.commons.io.filefilter.FalseFileFilter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.RobotMath;
import org.jmhsrobotics.frc2023.Constants.ArenaConstants;
import org.jmhsrobotics.frc2023.Constants.AutoConstants;
import org.jmhsrobotics.frc2023.Constants.RobotConstants;
import org.jmhsrobotics.frc2023.RobotMath.DiminishingAverageHandler;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDManager;

public class AutoBalance extends CommandBase {
	private Drivetrain drivetrain;
	private double moveLimit;
	private DiminishingAverageHandler robotPitchHandler;
	private DiminishingAverageHandler robotPitchAngVelHandler;
	private PIDController pidController;
	private PIDController pitchPIDController;
	private boolean isBalancing = false;
	private boolean onChargeStation = false;
	private Pose2d chargeCenterPosition;
	private Pose2d balancedPosition;
	private boolean atLimit = false;
	private Pose2d limitPosition;
	private boolean backwards;
	private boolean hasBalanced = false;
	private boolean hasBeenOnChargeStation = false;
	private boolean hasReachedBalancedSetpoint = false;

	LEDSubsystem.LEDStrip strip = LEDManager.STRIP0.strip;

	public AutoBalance(Drivetrain drivetrain, boolean backwards, LEDSubsystem ledSubsystem) {
		this.backwards = backwards;
		this.drivetrain = drivetrain;
		robotPitchHandler = new DiminishingAverageHandler(0.5);
		robotPitchAngVelHandler = new DiminishingAverageHandler(0.5);
		pidController = new PIDController(Constants.driveports.getBalancingPID().kp,
				Constants.driveports.getBalancingPID().ki, Constants.driveports.getBalancingPID().kd);
		pidController.setTolerance(0.02, 0.01);
		pitchPIDController = new PIDController(0.008, 0.0002, 0.0);
		addRequirements(drivetrain, ledSubsystem);
	}

	@Override
	public void initialize() {
		this.moveLimit = ArenaConstants.kchargeStationLengthMeters / 2;

		this.hasBalanced = false;

		pidController.reset();
		pitchPIDController.reset();

		System.out.println("KP: " + pidController.getP());
		System.out.println("KI: " + pidController.getI());
		System.out.println("KD: " + pidController.getD());
	}

	@Override
	public void execute() {
		// Command flow:
		// Command started with the back of the robot facing the charge station
		// (for possible future localization using apriltags)
		//
		// Robot should drive backwards until the robot reaches an angle that
		// tells whether it is on the charge station (charge station side sections are
		// 11 degrees,
		// middle is 15 degrees when tipped)
		//
		// The robot should then record its position, to make sure it doesn't drive back
		// off the other side.
		//
		// The robot should then creep backwards if it is tipped forwards,
		// and forwards if it is tipped backwards, until it reaches roughly level.
		// (there is a high threshold for level in this code as we don't want the robot
		// moving while it is in motion)
		//
		// If it reaches level, it should activate a PID to hold that position, unless
		// it stops being level.
		//
		// If it reaches a a limit, it should hold its position until it either needs to
		// move the other direction,
		// or the robot is level, at which point it will also hold position.
		//
		// This command will never complete manually, as it needs to hold its position.

		// spotless:off

		double pitchAngVel = this.robotPitchAngVelHandler.feed(
			(drivetrain.getPitch() - this.robotPitchHandler.get()) 
				/ RobotConstants.secondsPerTick
		);
		
		// spotless:on

		double pitch = this.robotPitchHandler.feed(drivetrain.getPitch());

		if (!this.hasBeenOnChargeStation/* !hasReachedChargeStation() */) {
			strip.setSolidColor(Color.kWhite);
			this.drivetrain.setCurvatureDrive(
					backwards ? -1 * AutoConstants.climbChargeStationSpeed : 1 * AutoConstants.climbChargeStationSpeed,
					0, false);
			// System.out.println(
			// backwards ? -1 * AutoConstants.climbChargeStationSpeed : 1 *
			// AutoConstants.climbChargeStationSpeed);

			this.hasBeenOnChargeStation = !RobotMath.approximatelyZero(pitch, 3.0);
		} else {
			if (Math.abs(pitchAngVel) > 50/* RobotMath.approximatelyZero(pitch, AutoConstants.balancedAngle) */) {

				if (!this.isBalancing) {
					this.hasBalanced = true;
					this.isBalancing = true;
					this.atLimit = false;
					this.balancedPosition = this.drivetrain.getPose();
					this.pidController.reset();
					this.pidController.setSetpoint(0);
				}
				strip.setSolidColor(Color.kBlue);
				this.drivetrain.setCurvatureDrive(-Math.signum(pitchAngVel) * AutoConstants.balanceCreepSpeed, 0,
						false);
				// this.drivetrain.setCurvatureDrive(
				// this.pidController.calculate(getRelativeDistance(this.balancedPosition)), 0,
				// false);

			} else {

				if (RobotMath.approximatelyZero(pitch, AutoConstants.balancedAngle)) {
					this.drivetrain.stop();
					this.strip.setSolidColor(Color.kGreen);
					return;
				}

				this.hasBeenOnChargeStation = true;

				this.isBalancing = false;

				// System.out.println("PITCH: " + pitch);

				double pidSpeedOutput;

				if (this.hasBalanced) {
					// spotless:off
					if(this.hasReachedBalancedSetpoint || this.pidController.atSetpoint()) {
						this.hasReachedBalancedSetpoint = true;
						pidSpeedOutput = this.pitchPIDController.calculate(
							pitch
						);
						this.strip.setSolidColor(Color.kOrange);
					} else {
						pidSpeedOutput = this.pidController.calculate(
							this.getRelativeDistance(this.balancedPosition)
						);
					}

					double speed = MathUtil.clamp(pidSpeedOutput, -0.07, 0.07);

					System.out.println("SPEED: " + speed);

					this.drivetrain.setCurvatureDrive(
						speed,
						0, 
						false
					);

				} else {

					this.drivetrain.setCurvatureDrive(
						-AutoConstants.balanceCreepSpeed, 
						0, 
						false
					);

				}

				/*

				if (pitch < 0 && getRelativeDistance(chargeCenterPosition) > moveLimit) {
					strip.setSolidColor(Color.kRed);
					this.drivetrain.setCurvatureDrive(
							hasBalanced ? AutoConstants.fineAdjustSpeed : AutoConstants.balanceCreepSpeed, 0, false);
				} else if (pitch > 0 && getRelativeDistance(chargeCenterPosition) < moveLimit) {
					this.drivetrain.setCurvatureDrive(
							-1 * (hasBalanced ? AutoConstants.fineAdjustSpeed : AutoConstants.balanceCreepSpeed), 0,
							false);
				} else {
					if (!atLimit) {
						atLimit = true;
						limitPosition = this.drivetrain.getPose();
						this.pidController.setSetpoint(0);
						this.pidController.reset();
					}
					strip.setSolidColor(Color.kGreen);
					this.drivetrain.setCurvatureDrive(this.pidController.calculate(getRelativeDistance(limitPosition)),
							0, false);
					System.out.println("REL D: " + getRelativeDistance(limitPosition));
					System.out.println("OUTPT: " + this.pidController.calculate(getRelativeDistance(limitPosition)));
				}

				*/

				// spotless:on
			}
		}

	}

	private double getRelativeDistance(Pose2d position) {
		System.out.println("RELD: " + new Transform2d(position, this.drivetrain.getPose()).getTranslation().getX());
		return new Transform2d(position, this.drivetrain.getPose()).getTranslation().getX();
	}

	private boolean hasReachedChargeStation() {
		if (this.onChargeStation == false) {
			if (Math.abs(this.robotPitchHandler.get()) > AutoConstants.onChargeStationAngle) {
				this.onChargeStation = true;
				this.chargeCenterPosition = this.drivetrain.getPose().plus(new Transform2d(
						new Translation2d(-1 * AutoConstants.balanceCenterLimitFromInitialTip, 0.0), new Rotation2d()));
				return true;
			}
			return false;

		}
		return this.onChargeStation;
	}

	@Override
	public boolean isFinished() {
		return false;
	}

}
