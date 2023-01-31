// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.gripper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.subsystems.gripper.GripperIO.GripperIOInputs;

public class Gripper extends SubsystemBase {
  private final GripperIO io;
  private final GripperIOInputs inputs = new GripperIOInputs();

  public Gripper(GripperIO io) {
    this.io = io;
    io.setBrakeMode(false);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Gripper", inputs);
  }

  /** Run the gripper at the specified percentage. */
  public void runPercent(double percent) {
    io.setVoltage(percent * 12.0);
  }

  public void stop() {
    runPercent(0.0);
  }
}
