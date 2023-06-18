// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;

public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_2023P;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;

  public static boolean invalidRobotAlertSent = false;

  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        if (!invalidRobotAlertSent) {
          new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
              .set(true);
          invalidRobotAlertSent = true;
        }
        return RobotType.ROBOT_2023C;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023C:
      case ROBOT_2023P:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2023C, "/media/sda2/");

  public static enum RobotType {
    ROBOT_2023C,
    ROBOT_2023P,
    ROBOT_SIMBOT
  }

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  // Function to disable HAL interaction when running without native libs
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robot == RobotType.ROBOT_SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
      System.exit(1);
    }
  }
}
