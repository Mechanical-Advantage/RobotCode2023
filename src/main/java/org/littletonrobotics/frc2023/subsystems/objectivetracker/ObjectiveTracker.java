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

  public GamePiece gamePiece = GamePiece.CUBE; // The selected game piece for intaking and scoring
  public boolean lastIntakeFront =
      true; // Whether the last game piece was grabbed from the front or back of the robot
  public int selectedRow = 0; // The row of the selected target node
  public NodeLevel selectedLevel = NodeLevel.HYBRID; // The level of the selected target node

  public ObjectiveTracker(NodeSelectorIO selectorIO) {
    this.selectorIO = selectorIO;
  }

  @Override
  public void periodic() {
    selectorIO.updateInputs(selectorInputs);
    Logger.getInstance().processInputs("NodeSelector", selectorInputs);

    // Send selected game piece
    SmartDashboard.putBoolean("Cube Selected", gamePiece == GamePiece.CUBE);

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
        case HYBRID -> selected += 0;
        case MID -> selected += 9;
        case HIGH -> selected += 18;
      }
      selectorIO.setSelected(selected);
    }

    // Send current node as text
    {
      String text = "";
      switch (selectedLevel) {
        case HYBRID -> text += "HYBRID";
        case MID -> text += "MID";
        case HIGH -> text += "HIGH";
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

    // Log state
    Logger.getInstance().recordOutput("ObjectiveTracker/GamePiece", gamePiece.toString());
    Logger.getInstance().recordOutput("ObjectiveTracker/LastIntakeFront", lastIntakeFront);
    Logger.getInstance().recordOutput("ObjectiveTracker/SelectedRow", selectedRow);
    Logger.getInstance().recordOutput("ObjectiveTracker/SelectedLevel", selectedLevel.toString());
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
          case HYBRID -> {}
          case MID -> selectedLevel = NodeLevel.HYBRID;
          case HIGH -> selectedLevel = NodeLevel.MID;
        }
        break;

      case DOWN:
        switch (selectedLevel) {
          case HYBRID -> selectedLevel = NodeLevel.MID;
          case MID -> selectedLevel = NodeLevel.HIGH;
          case HIGH -> {}
        }
        break;
    }
  }

  /** Command factory to shift the selected node in the selector by one position. */
  public Command shiftNodeCommand(Direction direction) {
    return new InstantCommand(() -> shiftNode(direction))
        .andThen(
            Commands.waitSeconds(0.3),
            Commands.repeatingSequence(
                new InstantCommand(() -> shiftNode(direction)), new WaitCommand(0.1)))
        .ignoringDisable(true);
  }

  public static enum GamePiece {
    CUBE,
    CONE
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
