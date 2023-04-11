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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.frc2023.subsystems.leds.Leds;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class ObjectiveTracker extends VirtualSubsystem {
  private final NodeSelectorIO selectorIO;
  private final NodeSelectorIOInputsAutoLogged selectorInputs =
      new NodeSelectorIOInputsAutoLogged();
  public final Objective objective = new Objective();

  public static class Objective {
    public int nodeRow;
    public NodeLevel nodeLevel;
    public ConeOrientation coneOrientation;
    public boolean lastIntakeFront;

    public Objective(
        int nodeRow,
        NodeLevel nodeLevel,
        ConeOrientation coneOrientation,
        boolean lastIntakeFront) {
      this.nodeRow = nodeRow;
      this.nodeLevel = nodeLevel;
      this.coneOrientation = coneOrientation;
      this.lastIntakeFront = lastIntakeFront;
    }

    public Objective() {
      this(0, NodeLevel.HYBRID, ConeOrientation.UPRIGHT, false);
    }

    public boolean isConeNode() {
      return nodeLevel != NodeLevel.HYBRID
          && (nodeRow == 0
              || nodeRow == 2
              || nodeRow == 3
              || nodeRow == 5
              || nodeRow == 6
              || nodeRow == 8);
    }

    public ScoringSide getScoringSide() {
      if (nodeLevel == NodeLevel.HYBRID) {
        return ScoringSide.BACK;
      } else if (!isConeNode()) {
        return ScoringSide.EITHER;
      } else {
        return coneOrientation == ConeOrientation.TIPPED ^ lastIntakeFront
            ? ScoringSide.FRONT
            : ScoringSide.BACK;
      }
    }
  }

  public ObjectiveTracker(NodeSelectorIO selectorIO) {
    System.out.println("[Init] Creating ObjectiveTracker");
    this.selectorIO = selectorIO;
  }

  @Override
  public void periodic() {
    selectorIO.updateInputs(selectorInputs);
    Logger.getInstance().processInputs("NodeSelector", selectorInputs);

    // Read updates from node selector
    if (selectorInputs.selectedNode != -1) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        objective.nodeRow = 8 - ((int) selectorInputs.selectedNode % 9);
      } else {
        objective.nodeRow = (int) selectorInputs.selectedNode % 9;
      }
      if (selectorInputs.selectedNode < 9) {
        objective.nodeLevel = NodeLevel.HYBRID;
      } else if (selectorInputs.selectedNode < 18) {
        objective.nodeLevel = NodeLevel.MID;
      } else {
        objective.nodeLevel = NodeLevel.HIGH;
      }
      selectorInputs.selectedNode = -1;
    }
    if (selectorInputs.coneTipped != -1) {
      objective.coneOrientation =
          selectorInputs.coneTipped == 0 ? ConeOrientation.UPRIGHT : ConeOrientation.TIPPED;
      selectorInputs.coneTipped = -1;
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

    // Send cone orientation to selector
    selectorIO.setConeOrientation(objective.coneOrientation == ConeOrientation.TIPPED);

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

    // Send cone orientation to dashboard and LEDs
    SmartDashboard.putBoolean("Cone Tipped", objective.coneOrientation == ConeOrientation.TIPPED);
    Leds.getInstance().hpConeTipped = objective.coneOrientation == ConeOrientation.TIPPED;

    // Log state
    Logger.getInstance().recordOutput("ObjectiveTracker/NodeRow", objective.nodeRow);
    Logger.getInstance().recordOutput("ObjectiveTracker/NodeLevel", objective.nodeLevel.toString());
    Logger.getInstance()
        .recordOutput("ObjectiveTracker/ConeOrientation", objective.coneOrientation.toString());
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

  /** Command factory to toggle whether the cone is tipped. */
  public Command toggleConeOrientationCommand() {
    return Commands.runOnce(
            () -> {
              switch (objective.coneOrientation) {
                case UPRIGHT:
                  objective.coneOrientation = ConeOrientation.TIPPED;
                  break;
                case TIPPED:
                  objective.coneOrientation = ConeOrientation.UPRIGHT;
                  break;
              }
            })
        .ignoringDisable(true);
  }

  public static enum NodeLevel {
    HYBRID,
    MID,
    HIGH
  }

  public static enum ConeOrientation {
    UPRIGHT,
    TIPPED,
  }

  public static enum ScoringSide {
    FRONT,
    BACK,
    EITHER
  }

  public static enum Direction {
    LEFT,
    RIGHT,
    UP,
    DOWN
  }
}
