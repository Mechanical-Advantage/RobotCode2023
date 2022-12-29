// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;

public class RobotContainerTest {

  @Test
  public void createRobotContainer() {
    // Set joysticks to silence warnings
    LoggedDriverStation.getInstance().getJoystickData(0).xbox = true;
    LoggedDriverStation.getInstance().getJoystickData(1).xbox = true;
    LoggedDriverStation.getInstance().getJoystickData(2).name = "Generic   USB  Joystick";

    // Instantiate RobotContainer
    new RobotContainer();
  }
}
