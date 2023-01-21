// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls (either single or dual Xbox). */
public class HandheldOI {
  protected static final Trigger dummyTrigger = new Trigger(() -> false);

  public double getLeftDriveX() {
    return 0.0;
  }

  public double getLeftDriveY() {
    return 0.0;
  }

  public double getRightDriveX() {
    return 0.0;
  }

  public double getRightDriveY() {
    return 0.0;
  }

  public Trigger getDriverAssist() {
    return dummyTrigger;
  }

  public void setDriverRumble(double percent) {}

  public void setOperatorRumble(double percent) {}
}
