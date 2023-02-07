// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2023.commands;

import org.jmhsrobotics.frc2023.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase taxi(Drivetrain drivetrain) {
    return Commands.sequence();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
