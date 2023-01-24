// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIO.ArmIOInputs;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Arm extends CommandBase {
  private ArmIO io;
  private final ArmIOInputs armInputs = new ArmIOInputs();

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
    switch (Constants.getRobot())
    {
      case ROBOT_2023C:
        break;
      case ROBOT_SIMBOT:
        break;
      default:
        break;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
