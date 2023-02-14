// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.util.GeomUtil;

public class ScoreHybridWithReach extends SequentialCommandGroup {
  public static double minReach = 1.15;
  public static double maxReach = 1.5;
  public static double wristHeight = 0.3;
  public static double wristHorizontalDistance = 0.4;
  public static Rotation2d wristAngle = Rotation2d.fromDegrees(-15.0);

  private final Drive drive;
  private final ObjectiveTracker objectiveTracker;

  public ScoreHybridWithReach(
      Drive drive, Arm arm, Gripper gripper, ObjectiveTracker objectiveTracker) {
    this.drive = drive;
    this.objectiveTracker = objectiveTracker;

    // Set up commands
    var driveCommand = new DriveToPose(drive, this::getDriveTarget);
    var armCommand =
        arm.runPathCommand(this::getArmTarget)
            .andThen(Commands.run(() -> arm.runDirect(getArmTarget()), arm));
    addCommands(
        Commands.waitUntil(() -> arm.isTrajectoryFinished() && driveCommand.atGoal())
            .deadlineWith(driveCommand, armCommand)
            .andThen(gripper.ejectCommand())
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }

  private Translation2d getScoreTarget() {
    return FieldConstants.Grids.complexLowTranslations[objectiveTracker.selectedRow];
  }

  private double getScoreTargetDistance() {
    return MathUtil.clamp(
        getScoreTarget().getDistance(drive.getPose().getTranslation()), minReach, maxReach);
  }

  private Rotation2d getAngleToScoreTarget() {
    return getScoreTarget().minus(drive.getPose().getTranslation()).getAngle();
  }

  private boolean shouldScoreFront() {
    return drive.getPose().getRotation().minus(getAngleToScoreTarget()).getCos() > 0.0;
  }

  private Pose2d getDriveTarget() {
    var targetForward =
        new Pose2d(getScoreTarget(), getAngleToScoreTarget())
            .transformBy(GeomUtil.translationToTransform(-getScoreTargetDistance(), 0.0));
    if (shouldScoreFront()) {
      return targetForward;
    } else {
      return targetForward.transformBy(
          new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0)));
    }
  }

  private ArmPose getArmTarget() {
    return new ArmPose(
            new Translation2d(getScoreTargetDistance() - wristHorizontalDistance, wristHeight),
            wristAngle)
        .withFlip(!shouldScoreFront());
  }
}
