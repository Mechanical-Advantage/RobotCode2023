// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

/** Class for controlling the robot with two Xbox controllers. */
public class DualHandheldOI extends HandheldOI {
  private final XboxController driverController;
  private final XboxController operatorController;

  public DualHandheldOI(int driverPort, int operatorPort) {
    driverController = new XboxController(driverPort);
    operatorController = new XboxController(operatorPort);
  }

  @Override
  public double getLeftDriveX() {
    return driverController.getLeftX();
  }

  @Override
  public double getLeftDriveY() {
    return driverController.getLeftY() * -1;
  }

  @Override
  public double getRightDriveX() {
    return driverController.getRightX();
  }

  @Override
  public double getRightDriveY() {
    return driverController.getRightY() * -1;
  }

  @Override
  public void setDriverRumble(double percent) {
    driverController.setRumble(RumbleType.kRightRumble, percent);
  }

  @Override
  public void setOperatorRumble(double percent) {
    operatorController.setRumble(RumbleType.kRightRumble, percent);
  }
}
