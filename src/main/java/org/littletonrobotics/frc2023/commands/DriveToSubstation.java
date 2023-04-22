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
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class DriveToSubstation extends DriveToPose {
  public static final Pose2d singleSubstationPose =
      new Pose2d(
          FieldConstants.LoadingZone.singleSubstationTranslation.plus(new Translation2d(0.0, -0.7)),
          Rotation2d.fromDegrees(90.0));
  public static final double doubleSubstationX =
      FieldConstants.LoadingZone.doubleSubstationX - 0.26;
  public static final double doubleSubstationExtraX = 0.55;

  /** Automatically drives to the nearest substation. */
  public DriveToSubstation(Drive drive, Supplier<Boolean> useDouble) {
    super(
        drive,
        () -> {
          Pose2d nearestTarget;
          if (useDouble.get()) {
            double currentX = AllianceFlipUtil.apply(drive.getPose().getX());
            nearestTarget =
                new Pose2d(
                    Math.max(doubleSubstationX, currentX + doubleSubstationExtraX),
                    drive.getPose().getY(),
                    new Rotation2d());
          } else {
            nearestTarget = singleSubstationPose;
          }
          nearestTarget = AllianceFlipUtil.apply(nearestTarget);
          if (drive.getRotation().minus(nearestTarget.getRotation()).getCos() < 0.0) {
            nearestTarget =
                nearestTarget.transformBy(
                    new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0)));
          }
          return nearestTarget;
        });
  }
}
