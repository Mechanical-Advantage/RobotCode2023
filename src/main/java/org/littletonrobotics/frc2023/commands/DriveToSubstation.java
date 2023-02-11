// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class DriveToSubstation extends DriveToPose {
  public static final Pose2d singleSubstationPose =
      new Pose2d(
          FieldConstants.LoadingZone.singleSubstationTranslation.plus(new Translation2d(0.0, -0.7)),
          Rotation2d.fromDegrees(90.0));
  public static final Pose2d doubleSubstationLeftPose =
      new Pose2d(
          FieldConstants.LoadingZone.doubleSubstationX - 0.5,
          FieldConstants.LoadingZone.doubleSubstationCenterY + 0.65,
          new Rotation2d());
  public static final Pose2d doubleSubstationRightPose =
      new Pose2d(
          FieldConstants.LoadingZone.doubleSubstationX - 0.5,
          FieldConstants.LoadingZone.doubleSubstationCenterY - 0.65,
          new Rotation2d());

  /** Automatically drives to the nearest substation. */
  public DriveToSubstation(Drive drive) {
    super(
        drive,
        () -> {
          var nearestTarget =
              drive
                  .getPose()
                  .nearest(
                      List.of(
                          AllianceFlipUtil.apply(singleSubstationPose),
                          AllianceFlipUtil.apply(doubleSubstationLeftPose),
                          AllianceFlipUtil.apply(doubleSubstationRightPose)));
          if (drive.getRotation().minus(nearestTarget.getRotation()).getCos() < 0.0) {
            nearestTarget =
                nearestTarget.transformBy(
                    new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0)));
          }
          return nearestTarget;
        });
  }
}
