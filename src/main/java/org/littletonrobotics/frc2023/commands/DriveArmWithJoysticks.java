// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import java.util.function.Supplier;

import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveArmWithJoysticks extends CommandBase {
  private final Arm arm;
  private final Supplier<Double> axisSupplier;

  /** Creates a new DriveArmWithJoysticks. */
  public DriveArmWithJoysticks(Arm arm, Supplier<Double> axisSupplier) 
  {
    addRequirements(arm);
    this.arm = arm;
    this.axisSupplier = axisSupplier;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Logger.getInstance().recordOutput("ActiveCommands/DriveArmWithJoySticks", "true");

    double value = axisSupplier.get();
    double scaledValue = 0.0;
    if (Math.abs(value) > DriveWithJoysticks.deadband) {
      scaledValue = (Math.abs(value) - DriveWithJoysticks.deadband)
          / (1 - DriveWithJoysticks.deadband);
      scaledValue = Math.copySign(scaledValue * scaledValue, value);
    }
    arm.setVoltage(scaledValue * 12);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
