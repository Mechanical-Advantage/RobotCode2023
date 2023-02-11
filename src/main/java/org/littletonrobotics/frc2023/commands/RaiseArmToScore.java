// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class RaiseArmToScore extends CommandBase {
  private final Arm arm;
  private final Drive drive;
  private final ObjectiveTracker objectiveTracker;

  private ArmPose lastPose = null;

  /** Raises the arm to score on the selected node. */
  public RaiseArmToScore(Arm arm, Drive drive, ObjectiveTracker objectiveTracker) {
    addRequirements(arm);
    this.arm = arm;
    this.drive = drive;
    this.objectiveTracker = objectiveTracker;
  }

  @Override
  public void initialize() {
    // Force pose to update on first execute cycle
    lastPose = null;
  }

  @Override
  public void execute() {
    // Get pose based on game piece and level
    ArmPose pose = ArmPose.Preset.HOMED.getPose();
    if (objectiveTracker.gamePiece == GamePiece.CUBE) {
      switch (objectiveTracker.selectedLevel) {
        case HYBRID -> pose = ArmPose.Preset.SCORE_HYBRID.getPose();
        case MID -> pose = ArmPose.Preset.SCORE_MID_CUBE.getPose();
        case HIGH -> pose = ArmPose.Preset.SCORE_HIGH_CUBE.getPose();
      }
    } else {
      switch (objectiveTracker.selectedLevel) {
        case HYBRID -> pose = ArmPose.Preset.SCORE_HYBRID.getPose();
        case MID -> pose = ArmPose.Preset.SCORE_MID_CONE.getPose();
        case HIGH -> pose = ArmPose.Preset.SCORE_HIGH_CONE.getPose();
      }
    }

    // Flip pose if scoring from back
    pose = pose.withFlip(!shouldScoreFront(drive.getRotation(), objectiveTracker));

    // Go to pose if changed
    if (lastPose == null || !pose.equals(lastPose)) {
      arm.runPath(pose);
      lastPose = pose;
    }
  }

  @Override
  public boolean isFinished() {
    return lastPose != null && arm.isTrajectoryFinished();
  }

  /** Returns whether to score off of the front or back of the robot. */
  public static boolean shouldScoreFront(
      Rotation2d driveRotation, ObjectiveTracker objectiveTracker) {
    var unflippedRotation = AllianceFlipUtil.apply(driveRotation);
    if (objectiveTracker.gamePiece == GamePiece.CUBE) {
      // Choose nearest side
      return unflippedRotation.getCos() < 0.0;

    } else {
      // Choose the same side as the cone was grabbed
      return objectiveTracker.lastIntakeFront;
    }
  }
}
