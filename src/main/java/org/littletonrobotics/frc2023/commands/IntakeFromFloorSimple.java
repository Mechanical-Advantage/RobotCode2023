// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;

public class IntakeFromFloorSimple extends SequentialCommandGroup {
  /** Intakes from the floor directly in front of or behind the robot. */
  public IntakeFromFloorSimple(boolean isFront, Arm arm, Gripper gripper, Objective objective) {
    addCommands(
        arm.runPathCommand(
                () -> {
                  if (isFront) {
                    if (objective.gamePiece == GamePiece.CUBE) {
                      return ArmPose.Preset.FLOOR_FRONT_CUBE.getPose();
                    } else {
                      return ArmPose.Preset.FLOOR_FRONT_CONE.getPose();
                    }
                  } else {
                    if (objective.gamePiece == GamePiece.CUBE) {
                      return ArmPose.Preset.FLOOR_BACK_CUBE.getPose();
                    } else {
                      return ArmPose.Preset.FLOOR_BACK_CONE.getPose();
                    }
                  }
                })
            .alongWith(
                gripper.intakeCommand(), Commands.run(() -> objective.lastIntakeFront = isFront))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }
}
