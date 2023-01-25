package frc.robot.commands.vision;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.PhotonManager;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Rotates the robot to align to a identified target. */
public class AlignPeg extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final double kp = 0.1, ki = 0, kd = 0;
  private final double kangleTolerance=1;
  private final double kmaxTurnSpeed=.5;
  PIDController m_pid = new PIDController(kp, ki, kd);

  /**
   * Rotates the robot to align to a identified target.
   *
   * @param drivetrain The subsystem used by this command.
   */
  public AlignPeg(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PhotonManager.getInstance().mainCam.setLED(VisionLEDMode.kOn);
    PhotonManager.getInstance().mainCam.setPipelineIndex(1);
    m_pid.reset();
    m_pid.setTolerance(kangleTolerance);
    m_pid.setSetpoint(0);
    SmartDashboard.putData(m_pid);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     
     PhotonPipelineResult result = PhotonManager.getInstance().mainCam.getLatestResult();
     SmartDashboard.putBoolean("alignpeg1/hasTarget", result.hasTargets());
     if(!result.hasTargets()){ // Checks to see if there is a Vision target
      return;
     }
     

     double targetYaw = result.getBestTarget().getYaw(); // Yaw in degrees from Center of camera + is Right
     SmartDashboard.putNumber("Vision/Yaw", targetYaw);
     double output = MathUtil.clamp( m_pid.calculate(targetYaw), -kmaxTurnSpeed, kmaxTurnSpeed); // Camp Turn output between kmaxTurnSpeed
    
     SmartDashboard.putNumber("alignpeg1/output", output);
     SmartDashboard.putNumber("alignpeg1/yaw", targetYaw);
     
     //m_drivetrain.setCurvatureDrive(0, output, true); //Turn Robot
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PhotonManager.getInstance().mainCam.setLED(VisionLEDMode.kOff);
    // TODO: Reset to old Pipeline or driver mode?
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: End cmd if the robot is at the setpoint.
    return false;
  }
}