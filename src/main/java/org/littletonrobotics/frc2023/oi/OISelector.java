// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;

/**
 * Utility class for selecting the appropriate OI implementations based on the connected joysticks.
 */
public class OISelector {
  private static final String overrideName = "Generic   USB  Joystick";
  private static String[] lastJoystickNames = new String[] {null, null, null, null, null, null};
  private static final Alert noOverrideWarning =
      new Alert("No override controller connected.", AlertType.INFO);
  private static final Alert noHandheldWarning =
      new Alert("No handheld controller(s) connected.", AlertType.WARNING);

  private OISelector() {}

  /**
   * Returns whether the connected joysticks have changed since the last time this method was
   * called.
   */
  public static boolean didJoysticksChange() {
    boolean joysticksChanged = false;
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      String name = DriverStation.getJoystickName(port);
      if (!name.equals(lastJoystickNames[port])) {
        lastJoystickNames[port] = name;
        joysticksChanged = true;
      }
    }
    return joysticksChanged;
  }

  /**
   * Instantiates and returns an appropriate override OI object based on the connected joysticks.
   */
  public static OverrideOI findOverrideOI() {
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (DriverStation.getJoystickName(port).equals(overrideName)) {
        noOverrideWarning.set(false);
        return new OverrideOI(port);
      }
    }
    noOverrideWarning.set(true);
    return new OverrideOI();
  }

  /**
   * Instantiates and returns an appropriate handheld OI object (driver and operator) based on the
   * connected joysticks.
   */
  public static HandheldOI findHandheldOI() {
    Integer driverPort = null;
    Integer operatorPort = null;
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (DriverStation.getJoystickIsXbox(port)) {
        if (driverPort == null) {
          driverPort = port;
        } else if (operatorPort == null) {
          operatorPort = port;
        }
      }
    }

    if (operatorPort != null) {
      noHandheldWarning.set(false);
      return new DualHandheldOI(driverPort, operatorPort);
    } else if (driverPort != null) {
      noHandheldWarning.set(false);
      return new SingleHandheldOI(driverPort);
    } else {
      noHandheldWarning.set(true);
      return new HandheldOI();
    }
  }
}
