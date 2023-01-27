// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;

public class DriveArmWithJoysticks extends CommandBase {
  private final Arm arm;
  private final Supplier<Double> axisSupplier;

  /** Creates a new DriveArmWithJoysticks. */
  public DriveArmWithJoysticks(Arm arm, Supplier<Double> axisSupplier) {
    addRequirements(arm);
    this.arm = arm;
    this.axisSupplier = axisSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = axisSupplier.get();
    double scaledValue = 0.0;
    if (Math.abs(value) > DriveWithJoysticks.deadband) {
      scaledValue =
          (Math.abs(value) - DriveWithJoysticks.deadband) / (1 - DriveWithJoysticks.deadband);
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
