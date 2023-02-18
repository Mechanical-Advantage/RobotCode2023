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
  public final Objective objective = new Objective();

  public static class Objective {
    public int nodeRow; // The row of the selected target node
    public NodeLevel nodeLevel; // The level of the selected target node
    public GamePiece gamePiece; // The selected game piece for intaking and scoring
    public boolean
        lastIntakeFront; // Whether the last game piece was grabbed from the front of the robot

    public Objective(
        int nodeRow, NodeLevel nodeLevel, GamePiece gamePiece, boolean lastIntakeFront) {
      this.nodeRow = nodeRow;
      this.nodeLevel = nodeLevel;
      this.gamePiece = gamePiece;
      this.lastIntakeFront = lastIntakeFront;
    }

    public Objective() {
      this.nodeRow = 0;
      this.nodeLevel = NodeLevel.HYBRID;
      this.gamePiece = GamePiece.CUBE;
      lastIntakeFront = true;
    }
  }

  public ObjectiveTracker(NodeSelectorIO selectorIO) {
    this.selectorIO = selectorIO;
  }

  @Override
  public void periodic() {
    selectorIO.updateInputs(selectorInputs);
    Logger.getInstance().processInputs("NodeSelector", selectorInputs);

    // Send selected game piece
    SmartDashboard.putBoolean("Cube Selected", objective.gamePiece == GamePiece.CUBE);

    // Read updates from node selector
    if (selectorInputs.selected != -1) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        objective.nodeRow = 8 - ((int) selectorInputs.selected % 9);
      } else {
        objective.nodeRow = (int) selectorInputs.selected % 9;
      }
      if (selectorInputs.selected < 9) {
        objective.nodeLevel = NodeLevel.HYBRID;
      } else if (selectorInputs.selected < 18) {
        objective.nodeLevel = NodeLevel.MID;
      } else {
        objective.nodeLevel = NodeLevel.HIGH;
      }
      selectorInputs.selected = -1;
    }

    // Send current node to selector
    {
      int selected;
      if (DriverStation.getAlliance() == Alliance.Blue) {
        selected = 8 - objective.nodeRow;
      } else {
        selected = objective.nodeRow;
      }
      switch (objective.nodeLevel) {
        case HYBRID -> selected += 0;
        case MID -> selected += 9;
        case HIGH -> selected += 18;
      }
      selectorIO.setSelected(selected);
    }

    // Send current node as text
    {
      String text = "";
      switch (objective.nodeLevel) {
        case HYBRID -> text += "HYBRID";
        case MID -> text += "MID";
        case HIGH -> text += "HIGH";
      }
      text += ", ";
      if (objective.nodeRow < 3) {
        text += DriverStation.getAlliance() == Alliance.Red ? "LEFT" : "RIGHT";
      } else if (objective.nodeRow < 6) {
        text += "CO-OP";
      } else {
        text += DriverStation.getAlliance() == Alliance.Red ? "RIGHT" : "LEFT";
      }
      text += " grid, ";
      if (objective.nodeRow == 1 || objective.nodeRow == 4 || objective.nodeRow == 7) {
        text += objective.nodeLevel == NodeLevel.HYBRID ? "CENTER" : "CUBE";
      } else if (objective.nodeRow == 0 || objective.nodeRow == 3 || objective.nodeRow == 6) {
        text += DriverStation.getAlliance() == Alliance.Red ? "LEFT" : "RIGHT";
        text += objective.nodeLevel == NodeLevel.HYBRID ? "" : " CONE";
      } else {
        text += DriverStation.getAlliance() == Alliance.Red ? "RIGHT" : "LEFT";
        text += objective.nodeLevel == NodeLevel.HYBRID ? "" : " CONE";
      }
      text += " node";
      SmartDashboard.putString("Selected Node", text);
    }

    // Log state
    Logger.getInstance().recordOutput("ObjectiveTracker/NodeRow", objective.nodeRow);
    Logger.getInstance().recordOutput("ObjectiveTracker/NodeLevel", objective.nodeLevel.toString());
    Logger.getInstance().recordOutput("ObjectiveTracker/GamePiece", objective.gamePiece.toString());
    Logger.getInstance()
        .recordOutput("ObjectiveTracker/LastIntakeFront", objective.lastIntakeFront);
  }

  /** Shifts the selected node in the selector by one position. */
  public void shiftNode(Direction direction) {
    switch (direction) {
      case LEFT:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          if (objective.nodeRow < 8) {
            objective.nodeRow += 1;
          }
        } else {
          if (objective.nodeRow > 0) {
            objective.nodeRow -= 1;
          }
        }
        break;

      case RIGHT:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          if (objective.nodeRow > 0) {
            objective.nodeRow -= 1;
          }
        } else {
          if (objective.nodeRow < 8) {
            objective.nodeRow += 1;
          }
        }
        break;

      case UP:
        switch (objective.nodeLevel) {
          case HYBRID -> {}
          case MID -> objective.nodeLevel = NodeLevel.HYBRID;
          case HIGH -> objective.nodeLevel = NodeLevel.MID;
        }
        break;

      case DOWN:
        switch (objective.nodeLevel) {
          case HYBRID -> objective.nodeLevel = NodeLevel.MID;
          case MID -> objective.nodeLevel = NodeLevel.HIGH;
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
