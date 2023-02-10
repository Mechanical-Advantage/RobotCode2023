// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.objectivetracker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;

public class ObjectiveTracker extends SubsystemBase {
  private final NodeSelectorIO selectorIO;
  private final NodeSelectorIOInputsAutoLogged selectorInputs =
      new NodeSelectorIOInputsAutoLogged();

  private int selectedRow = 0;
  private NodeLevel selectedLevel = NodeLevel.HYBRID;

  public ObjectiveTracker(NodeSelectorIO selectorIO) {
    this.selectorIO = selectorIO;
  }

  @Override
  public void periodic() {
    selectorIO.updateInputs(selectorInputs);
    Logger.getInstance().processInputs("NodeSelector", selectorInputs);

    // Read updates from node selector
    if (selectorInputs.selected != -1) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        selectedRow = 8 - ((int) selectorInputs.selected % 9);
      } else {
        selectedRow = (int) selectorInputs.selected % 9;
      }
      if (selectorInputs.selected < 9) {
        selectedLevel = NodeLevel.HYBRID;
      } else if (selectorInputs.selected < 18) {
        selectedLevel = NodeLevel.MID;
      } else {
        selectedLevel = NodeLevel.HIGH;
      }
      selectorInputs.selected = -1;
    }

    // Send current node to selector
    {
      int selected;
      if (DriverStation.getAlliance() == Alliance.Blue) {
        selected = 8 - selectedRow;
      } else {
        selected = selectedRow;
      }
      switch (selectedLevel) {
        case HYBRID:
          selected += 0;
          break;
        case MID:
          selected += 9;
          break;
        case HIGH:
          selected += 18;
          break;
      }
      selectorIO.setSelected(selected);
    }

    // Log current node as text
    {
      String text = "";
      switch (selectedLevel) {
        case HYBRID:
          text += "HYBRID";
          break;
        case MID:
          text += "MID";
          break;
        case HIGH:
          text += "HIGH";
          break;
      }
      text += ", ";
      if (selectedRow < 3) {
        text += DriverStation.getAlliance() == Alliance.Red ? "LEFT" : "RIGHT";
      } else if (selectedRow < 6) {
        text += "CO-OP";
      } else {
        text += DriverStation.getAlliance() == Alliance.Red ? "RIGHT" : "LEFT";
      }
      text += " grid, ";
      if (selectedRow == 1 || selectedRow == 4 || selectedRow == 7) {
        text += selectedLevel == NodeLevel.HYBRID ? "CENTER" : "CUBE";
      } else if (selectedRow == 0 || selectedRow == 3 || selectedRow == 6) {
        text += DriverStation.getAlliance() == Alliance.Red ? "LEFT" : "RIGHT";
        text += selectedLevel == NodeLevel.HYBRID ? "" : " CONE";
      } else {
        text += DriverStation.getAlliance() == Alliance.Red ? "RIGHT" : "LEFT";
        text += selectedLevel == NodeLevel.HYBRID ? "" : " CONE";
      }
      text += " node";
      SmartDashboard.putString("Selected Node", text);
    }
  }

  /** Shifts the selected node in the selector by one position. */
  public void shiftNode(Direction direction) {
    switch (direction) {
      case LEFT:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          if (selectedRow < 8) {
            selectedRow += 1;
          }
        } else {
          if (selectedRow > 0) {
            selectedRow -= 1;
          }
        }
        break;

      case RIGHT:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          if (selectedRow > 0) {
            selectedRow -= 1;
          }
        } else {
          if (selectedRow < 8) {
            selectedRow += 1;
          }
        }
        break;

      case UP:
        switch (selectedLevel) {
          case HYBRID:
            break;
          case MID:
            selectedLevel = NodeLevel.HYBRID;
            break;
          case HIGH:
            selectedLevel = NodeLevel.MID;
            break;
        }
        break;

      case DOWN:
        switch (selectedLevel) {
          case HYBRID:
            selectedLevel = NodeLevel.MID;
            break;
          case MID:
            selectedLevel = NodeLevel.HIGH;
            break;
          case HIGH:
            break;
        }
        break;
    }
  }

  /** Command factory to shift the selected node in the selector by one position. */
  public Command shiftNodeCommand(Direction direction) {
    return new InstantCommand(() -> shiftNode(direction))
        .andThen(
            Commands.waitSeconds(0.5),
            Commands.repeatingSequence(
                new InstantCommand(() -> shiftNode(direction)), new WaitCommand(0.1)))
        .ignoringDisable(true);
  }

  public static enum NodeLevel {
    HYBRID,
    MID,
    HIGH
  }

  public static enum Direction {
    LEFT,
    RIGHT,
    UP,
    DOWN
  }
}
