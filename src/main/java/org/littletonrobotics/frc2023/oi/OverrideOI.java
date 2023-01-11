// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj.Joystick;

/** Class for the override switches on the OI console. */
public class OverrideOI {
  private Joystick overrides;

  /** Creates a dummy set of overrides if controller is not available. */
  public OverrideOI() {}

  /** Creates a set of overrides using the given controller port. */
  public OverrideOI(int port) {
    overrides = new Joystick(port);
  }

  public boolean getRobotRelative() {
    if (overrides == null) {
      return false;
    }
    return overrides.getRawButton(1);
  }
}
