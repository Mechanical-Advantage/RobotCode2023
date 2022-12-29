// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI extends HandheldOI {
  private final XboxController controller;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getLeftDriveX() {
    return controller.getLeftX();
  }

  @Override
  public double getLeftDriveY() {
    return controller.getLeftY() * -1;
  }

  @Override
  public double getRightDriveX() {
    return controller.getRightX();
  }

  @Override
  public double getRightDriveY() {
    return controller.getRightY() * -1;
  }

  @Override
  public void setDriverRumble(double percent) {
    controller.setRumble(RumbleType.kRightRumble, percent);
  }

  @Override
  public void setOperatorRumble(double percent) {
    controller.setRumble(RumbleType.kRightRumble, percent);
  }
}
