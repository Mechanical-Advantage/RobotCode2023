// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Class for controlling the robot with two Xbox controllers. */
public class DualHandheldOI extends HandheldOI {
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  public DualHandheldOI(int driverPort, int operatorPort) {
    driverController = new CommandXboxController(driverPort);
    operatorController = new CommandXboxController(operatorPort);
  }

  @Override
  public double getLeftDriveX() {
    return -driverController.getLeftY();
  }

  @Override
  public double getLeftDriveY() {
    return -driverController.getLeftX();
  }

  @Override
  public double getRightDriveX() {
    return -driverController.getRightY();
  }

  @Override
  public double getRightDriveY() {
    return -driverController.getRightX();
  }

  @Override
  public void setDriverRumble(double percent) {
    driverController.getHID().setRumble(RumbleType.kRightRumble, percent);
  }

  @Override
  public void setOperatorRumble(double percent) {
    operatorController.getHID().setRumble(RumbleType.kRightRumble, percent);
  }
}
