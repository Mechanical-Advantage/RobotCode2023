// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Optional;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;

public class FollowDemoTag extends SequentialCommandGroup {
  private static final LoggedTunableNumber targetDistance =
      new LoggedTunableNumber("TrackDemoTag/TargetDistance", 2.0);

  private static final Translation3d upTranslation = new Translation3d(0.0, 0.0, 1.0);
  private static final Translation3d leftTranslation = new Translation3d(0.0, 1.0, 0.0);
  private static final Translation3d downTranslation = new Translation3d(0.0, 0.0, -1.0);
  private static final Translation3d rightTranslation = new Translation3d(0.0, -1.0, 0.0);

  public FollowDemoTag(Drive drive, AprilTagVision aprilTagVision) {
    addCommands(
        Commands.runOnce(() -> aprilTagVision.setVisionUpdatesEnabled(false))
            .andThen(
                new DriveToPose(
                    drive,
                    true,
                    () -> {
                      Optional<Pose3d> tagPose = aprilTagVision.getDemoTagPose();
                      if (tagPose.isEmpty()) {
                        return drive.getPose();
                      }

                      // Determine tag rotation
                      double maxZ = 0.0;
                      int maxIndex = 0;
                      int index = 0;
                      for (var translation :
                          new Translation3d[] {
                            upTranslation, leftTranslation, downTranslation, rightTranslation
                          }) {
                        double z =
                            tagPose
                                .get()
                                .transformBy(new Transform3d(translation, new Rotation3d()))
                                .getZ();
                        if (z > maxZ) {
                          maxZ = z;
                          maxIndex = index;
                        }
                        index++;
                      }
                      Rotation2d robotRotation = new Rotation2d(Math.PI + Math.PI / 2.0 * maxIndex);

                      // Calculate robot pose
                      return tagPose
                          .get()
                          .toPose2d()
                          .transformBy(
                              new Transform2d(
                                  new Translation2d(targetDistance.get(), 0.0), robotRotation));
                    }))
            .finallyDo((boolean interrupted) -> aprilTagVision.setVisionUpdatesEnabled(true)));
  }
}
