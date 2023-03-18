package org.jmhsrobotics.frc2023.commands.auto;

import edu.wpi.first.math.MathUtil;

//import org.apache.commons.io.filefilter.FalseFileFilter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.jmhsrobotics.frc2023.Constants;
import org.jmhsrobotics.frc2023.RobotContainer;
import org.jmhsrobotics.frc2023.RobotMath;
import org.jmhsrobotics.frc2023.Constants.AutoConstants;
import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDAnimation;
import org.jmhsrobotics.frc2023.util.LEDs.LEDSubsystem.LEDManager;

public class AutoBalance extends CommandBase {

	private Drivetrain drivetrain;

	private Timer timeoutTimer;

	// private double moveLimit;
	// private DiminishingAverageHandler robotPitchHandler;
	// private DiminishingAverageHandler robotPitchAngVelHandler;
	private PIDController pidController;
	private PIDController pitchPIDController;
	private boolean isBalancing = false;
	private boolean isTipping = false;
	// private boolean onChargeStation = false;
	// private Pose2d chargeCenterPosition;
	// private Pose2d balancedPosition;
	// private boolean atLimit = false;
	// private Pose2d limitPosition;
	private boolean backwards;
	private boolean hasTipped = false;
	private boolean hasBeenOnChargeStation = false;
	// private boolean hasReachedBalancedSetpoint = false;

	LEDSubsystem.LEDStrip strip = LEDManager.STRIP0.strip;

	private LEDAnimation blinkingAnim = strip.blinkingAnimation(0.125, Color.kRed, Color.kOrange, Color.kYellow,
			Color.kGreen, Color.kBlue, Color.kPurple);

	private LEDAnimation redBlueAnim = strip.fadeTwoAnimation(1, 20, Color.kRed, Color.kBlue);

	public AutoBalance(Drivetrain drivetrain, boolean backwards, LEDSubsystem ledSubsystem) {
		this.backwards = backwards;
		this.drivetrain = drivetrain;
		this.timeoutTimer = new Timer();
		// this.robotPitchHandler = new DiminishingAverageHandler(0.5);
		// robotPitchAngVelHandler = new DiminishingAverageHandler(0.5);
		this.pidController = new PIDController(Constants.driveports.getBalancingPID().kp,
				Constants.driveports.getBalancingPID().ki, Constants.driveports.getBalancingPID().kd);

		this.pidController.setTolerance(0.02, 0.01);
		this.pitchPIDController = new PIDController(0.008, 0.0002, 0.0);
		addRequirements(drivetrain, ledSubsystem);
	}

	@Override
	public void initialize() {
		// this.moveLimit = ArenaConstants.kchargeStationLengthMeters / 2;

		this.hasTipped = false;
		this.isTipping = false;

		this.hasBeenOnChargeStation = false;

		this.isBalancing = false;

		this.timeoutTimer.reset();
		this.timeoutTimer.start();

		// pidController.reset();
		this.pitchPIDController.reset();

		// System.out.println("KP: " + pidController.getP());
		// System.out.println("KI: " + pidController.getI());
		// System.out.println("KD: " + pidController.getD());
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
		// If it reaches level, it should activate++ a PID to hold that position, unless
		// it stops being level.
		//
		// If it reaches a a limit, it should hold its position until it either needs to
		// move the other direction,
		// or the robot is level, at which point it will also hold position.
		//
		// This command will never complete manually, as it needs to hold its position.
		double pitch = RobotContainer.getTelemetry().getPitch();
		double pitchAngVel = RobotContainer.getTelemetry().getPitchVelocity();

		// IF THE ROBOT HAS NOT BEEN ON THE CHARGE STATION YET
		if (!this.hasBeenOnChargeStation) {

			// strip.setSolidColor(Color.kWhite);
			this.blinkingAnim.step();

			// spotless:off
            this.drivetrain.setCurvatureDrive(
                (backwards ? -1.0 : 1.0) * AutoConstants.climbChargeStationSpeed,
                0, false
            );
            // spotless:on

			// System.out.println(
			// backwards ? -1 * AutoConstants.climbChargeStationSpeed : 1 *
			// AutoConstants.climbChargeStationSpeed);

			// IF THE ROBOT IS AT AN ANGLE OF > 3.0 OR < 3.0 DEGREES
			// i.e. if the robot is angling up or down
			// REGISTER THE ROBOT AS HAVING BEEN ON THE CHARGING STATION
			this.hasBeenOnChargeStation = !RobotMath.approximatelyZero(pitch, 3.0);
		} else {

			// IF THE ROBOT IS FALLING FORWARD OR BACKWARD (The loading station is tipping
			// over!)
			// TODO: tune this on the field (determines how sensitive the robot is to
			// tipping) higher value -> LESS sensitive
			if (Math.abs(pitchAngVel) > 40) {

				if (!this.isTipping) {
					this.hasTipped = true;
					this.isTipping = true;
					// this.atLimit = false;
					// this.balancedPosition = this.drivetrain.getPose();
					// this.pidController.reset();
					// this.pidController.setSetpoint(0);
				}

				// strip.setSolidColor(Color.kBlue);
				this.redBlueAnim.step();

				// this.drivetrain.stop();

				// Drive toward the center
				// TODO: tune this (how hard the robot reverses when tipping)
				this.drivetrain.setCurvatureDrive(-Math.signum(pitchAngVel) * 0.04, 0, false);

			} else {

				// THE CHARGING STATION IS NOT TIPPING

				// IS BALANCING
				if (RobotMath.approximatelyZero(pitch, AutoConstants.balancedAngle)) {
					this.drivetrain.stop();
					this.strip.setSolidColor(Color.kGreen);
					this.isBalancing = true;
					return;
				}

				// IF NOT BALANCED

				this.hasBeenOnChargeStation = true;

				this.isTipping = false;

				// System.out.println("PITCH: " + pitch);

				// IF THE ROBOT HAS BALANCED BEFORE
				// i.e. if we are near-ish to the center of balance.
				if (this.hasTipped) {
					// spotless:off
                    // if(this.hasReachedBalancedSetpoint || this.pidController.atSetpoint()) {
                    //  this.hasReachedBalancedSetpoint = true;
                    //  pidSpeedOutput = this.pitchPIDController.calculate(
                    //      pitch
                    //  );
                    //  this.strip.setSolidColor(Color.kOrange);
                    // }

                    double pidSpeedOutput = this.pitchPIDController.calculate(
                        pitch
                    );

                    this.strip.setSolidColor(Color.kOrange);

                    // } else {
                    //  pidSpeedOutput = this.pidController.calculate(
                    //      this.getRelativeDistance(this.balancedPosition)
                    //  );
                    // }

					// TODO: maybe tune this (how fast the pitch PID is when recovering from tipping)
                    double speed = MathUtil.clamp(pidSpeedOutput, -0.8, 0.8);

                    System.out.println("SPEED: " + speed);

                    this.drivetrain.setCurvatureDrive(
                        speed,
                        0, 
                        false
                    );

                } else {

                    // THE ROBOT IS ON THE CHARGE STATION WITHOUT HAVING BALANCED YET
                    // The robot is creeping up the station for the first time

					// TODO: tune this (important) how fast the robot creeps up the charge station at the start
                    this.drivetrain.setCurvatureDrive(
                        -0.33, 
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

		this.strip.sendData();

	}

	// private double getRelativeDistance(Pose2d position) {
	// System.out.println("RELD: " + new Transform2d(position,
	// this.drivetrain.getPose()).getTranslation().getX());
	// return new Transform2d(position,
	// this.drivetrain.getPose()).getTranslation().getX();
	// }

	// private boolean hasReachedChargeStation() {
	// if (this.onChargeStation == false) {
	// if (Math.abs(this.robotPitchHandler.get()) >
	// AutoConstants.onChargeStationAngle) {
	// this.onChargeStation = true;
	// this.chargeCenterPosition = this.drivetrain.getPose().plus(new Transform2d(
	// new Translation2d(-1 * AutoConstants.balanceCenterLimitFromInitialTip, 0.0),
	// new Rotation2d()));
	// return true;
	// }
	// return false;

	// }
	// return this.onChargeStation;
	// }

	@Override
	public void end(boolean interrupted) {
		this.timeoutTimer.stop();
	}

	@Override
	public boolean isFinished() {
		// spotless:off
        return this.isBalancing 
			|| this.timeoutTimer.hasElapsed(AutoConstants.autoBalanceTimeoutSeconds);
        // spotless:on
	}

}
