// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.oi.HandheldOI;
import org.littletonrobotics.frc2023.oi.OISelector;
import org.littletonrobotics.frc2023.oi.OverrideOI;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Subsystems

  // OI objects
  private OverrideOI overrideOI = new OverrideOI();
  private HandheldOI handheldOI = new HandheldOI();

  // Choosers
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public RobotContainer() {
    // Check if flash should be burned
    SparkMaxBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
          break;
        case ROBOT_2023P:
          break;
        case ROBOT_SIMBOT:
          break;
      }
    }

    // Instantiate missing subsystems

    // Set up subsystems

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", null);

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }

    // Instantiate OI classes and bind buttons
    updateOI();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    overrideOI = OISelector.findOverrideOI();
    handheldOI = OISelector.findHandheldOI();

    // *** DRIVER CONTROLS ***

    // *** OPERATOR CONTROLS ***
  }

  /** Passes the autonomous command to the {@link Robot} class. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
