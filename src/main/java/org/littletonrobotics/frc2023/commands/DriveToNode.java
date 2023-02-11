// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class DriveToNode extends DriveToPose {
  public static final double scorePositionX = FieldConstants.Grids.outerX + 0.4;

  public DriveToNode(Drive drive, ObjectiveTracker objectiveTracker) {
    super(
        drive,
        () -> {
          Translation2d targetTranslation = new Translation2d();
          switch (objectiveTracker.selectedLevel) {
            case HYBRID:
              targetTranslation =
                  FieldConstants.Grids.complexLowTranslations[objectiveTracker.selectedRow];
              break;
            case MID:
              targetTranslation =
                  FieldConstants.Grids.midTranslations[objectiveTracker.selectedRow];
              break;
            case HIGH:
              targetTranslation =
                  FieldConstants.Grids.highTranslations[objectiveTracker.selectedRow];
              break;
          }

          targetTranslation = new Translation2d(scorePositionX, targetTranslation.getY());
          var scoreFront = RaiseArmToScore.shouldScoreFront(drive.getRotation(), objectiveTracker);
          return AllianceFlipUtil.apply(
              new Pose2d(
                  targetTranslation,
                  scoreFront ? Rotation2d.fromDegrees(180.0) : new Rotation2d()));
        });
  }
}
