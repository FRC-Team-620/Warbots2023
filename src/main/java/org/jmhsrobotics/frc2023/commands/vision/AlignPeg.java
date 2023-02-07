package org.jmhsrobotics.frc2023.commands.vision;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import org.jmhsrobotics.frc2023.util.vision.PhotonManager;

// import org.apache.commons.io.filefilter.FalseFileFilter;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Rotates the robot to align to a identified target. */
public class AlignPeg extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final double kp = 0.014, ki = 0.01, kd = .001;
    private final double kangleTolerance = 2;
    private final double kmaxTurnSpeed = .5;
    private int lastPipeline = 0;

    PIDController m_pid = new PIDController(kp, ki, kd);

    /**
     * Rotates the robot to align to a identified target.
     *
     * @param drivetrain
     *                   The subsystem used by this command.
     */
    public AlignPeg(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        SmartDashboard.putData("alignpeg1/pid", m_pid);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        PhotonManager.getInstance().mainCam.setLED(VisionLEDMode.kOn);
        PhotonManager.getInstance().mainCam.setPipelineIndex(0);
        lastPipeline = PhotonManager.getInstance().mainCam.getPipelineIndex();
        m_pid.reset();
        m_pid.setTolerance(kangleTolerance, 1);
        m_pid.enableContinuousInput(-180, 180);
        // Enables continuous input on a range from -180 to 180
        double currentHeading = m_drivetrain.getPose().getRotation().getDegrees();

        m_pid.setSetpoint(currentHeading);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double currentHeading = m_drivetrain.getPose().getRotation().getDegrees();
        // double targetHeading = m_pid.getSetpoint();

        PhotonPipelineResult result = PhotonManager.getInstance().mainCam.getLatestResult();

        SmartDashboard.putNumber("alignpeg1/hasTarget", result.hasTargets() ? 1 : -1);
        System.out.println(result.hasTargets());
        if (result.hasTargets()) {
            double targetHeading = currentHeading - result.getBestTarget().getYaw();
            SmartDashboard.putNumber("alignpeg1/visionYaw", result.getBestTarget().getYaw());
            m_pid.setSetpoint(targetHeading);
        }
        double output = -MathUtil.clamp(m_pid.calculate(currentHeading), -kmaxTurnSpeed, kmaxTurnSpeed); // Clamp Turn
                                                                                                         // // output
                                                                                                         // between
                                                                                                         // kmaxTurnSpeed
        SmartDashboard.putNumber("alignpeg1/setpoint", m_pid.getSetpoint());
        SmartDashboard.putNumber("alignpeg1/output", output);
        SmartDashboard.putNumber("alignpeg1/robotyaw", currentHeading);

        m_drivetrain.setCurvatureDrive(0, MathUtil.clamp(output, -.1, .1), true); // Update Drivetrain
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        PhotonManager.getInstance().mainCam.setLED(VisionLEDMode.kOff);
        PhotonManager.getInstance().mainCam.setPipelineIndex(0);

        // TODO: Reset to old Pipeline or driver mode?
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return m_pid.atSetpoint();
        return false;
    }
}
