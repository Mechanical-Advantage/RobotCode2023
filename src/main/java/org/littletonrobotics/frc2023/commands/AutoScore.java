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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.GeomUtil;

public class AutoScore extends SequentialCommandGroup {
  public static final double minDriveX = FieldConstants.Grids.outerX + 0.5;
  public static final double minDriveY = 0.5;
  public static final double maxDriveY = FieldConstants.Community.leftY - 0.5;
  public static final double minArmExtension = 0.6;

  public static final Translation2d hybridRelativePosition = new Translation2d(-0.2, 0.6);
  public static final Rotation2d hybridWristAngle = Rotation2d.fromDegrees(-60.0);
  public static final Translation2d midCubeRelativePosition = new Translation2d(-0.5, 0.5);
  public static final Rotation2d midCubeWristAngle = Rotation2d.fromDegrees(-30.0);
  public static final Translation2d midConeRelativePosition = new Translation2d(-0.25, 0.15);
  public static final Rotation2d midConeWristAngle = Rotation2d.fromDegrees(30.0);
  public static final Translation2d highCubeRelativePosition = new Translation2d(-0.5, 0.5);
  public static final Rotation2d highCubeWristAngle = Rotation2d.fromDegrees(-30.0);
  public static final Translation2d highConeRelativePosition = new Translation2d(-0.25, 0.15);
  public static final Rotation2d highConeWristAngle = Rotation2d.fromDegrees(30.0);

  private final Drive drive;
  private final Arm arm;
  private final ObjectiveTracker objectiveTracker;

  public AutoScore(
      Drive drive,
      Arm arm,
      Gripper gripper,
      ObjectiveTracker objectiveTracker,
      Supplier<Boolean> autoAlignDisable,
      Supplier<Boolean> autoPlaceDisable,
      Supplier<Boolean> reachScoreDisable) {
    this.drive = drive;
    this.arm = arm;
    this.objectiveTracker = objectiveTracker;

    // Set up commands
    Supplier<Pose2d> driveTargetSupplier = () -> getDriveTarget(!reachScoreDisable.get());
    Supplier<ArmPose> armTargetSupplier =
        () -> getArmTarget(getDriveTarget(!reachScoreDisable.get()));
    var driveCommand = new DriveToPose(drive, driveTargetSupplier);
    var armCommand =
        arm.runPathCommand(armTargetSupplier)
            .andThen(Commands.run(() -> arm.runDirect(armTargetSupplier.get()), arm));
    addCommands(
        Commands.waitUntil(() -> arm.isTrajectoryFinished() && driveCommand.atGoal())
            .deadlineWith(driveCommand, armCommand)
            .andThen(gripper.ejectCommand())
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }

  /** Returns whether to score off of the front or back of the robot. */
  private boolean shouldScoreFront() {
    var currentPose = AllianceFlipUtil.apply(drive.getPose());
    var nodeTranslation = GeomUtil.translation3dTo2dXY(getNodeTranslation());
    var relativeRotation =
        nodeTranslation
            .minus(currentPose.getTranslation())
            .getAngle()
            .minus(currentPose.getRotation());
    if (objectiveTracker.selectedLevel == NodeLevel.HYBRID
        || objectiveTracker.gamePiece == GamePiece.CUBE) {
      // Choose nearest side
      return relativeRotation.getCos() > 0.0;

    } else {
      // Choose the same side as the cone was grabbed
      return objectiveTracker.lastIntakeFront;
    }
  }

  /** Returns the best drive target for the selected node. */
  private Pose2d getDriveTarget(boolean allowReach) {
    var nodeTranslation = getNodeTranslation();
    var currentPose = AllianceFlipUtil.apply(drive.getPose()); // Unflipped
    var relativeArmPosition = getRelativeArmPosition();

    // Calculate drive distance
    double minDistance = minArmExtension - relativeArmPosition.getX();
    double maxDistance =
        arm.calcMaxReachAtHeight(nodeTranslation.getZ() + relativeArmPosition.getY())
            - relativeArmPosition.getX();
    double distanceFromNode =
        MathUtil.clamp(
            currentPose.getTranslation().getDistance(GeomUtil.translation3dTo2dXY(nodeTranslation)),
            minDistance,
            maxDistance);

    // If reach not allowed, return target at minimum distance
    if (!allowReach) {
      return AllianceFlipUtil.apply(
          new Pose2d(
              Math.max(minDriveX, nodeTranslation.getX() + minDistance),
              nodeTranslation.getY(),
              Rotation2d.fromDegrees(shouldScoreFront() ? 180.0 : 0.0)));
    }

    // Calculate angle from node
    var angleFromNode =
        currentPose
            .getTranslation()
            .minus(GeomUtil.translation3dTo2dXY(nodeTranslation))
            .getAngle();
    var maxAngleFromNode =
        new Rotation2d(Math.acos((minDriveX - nodeTranslation.getX()) / maxDistance));
    if (angleFromNode.getRadians() > maxAngleFromNode.getRadians()) {
      angleFromNode = maxAngleFromNode;
    }
    if (angleFromNode.getRadians() < -maxAngleFromNode.getRadians()) {
      angleFromNode = maxAngleFromNode.unaryMinus();
    }

    // Increase distance if below min x
    double minDistanceAtAngle = (minDriveX - nodeTranslation.getX()) / angleFromNode.getCos();
    if (distanceFromNode < minDistanceAtAngle) {
      distanceFromNode = minDistanceAtAngle;
    }

    // Get drive pose
    var driveTranslation =
        new Pose2d(GeomUtil.translation3dTo2dXY(nodeTranslation), angleFromNode)
            .transformBy(GeomUtil.translationToTransform(distanceFromNode, 0.0))
            .getTranslation();
    driveTranslation =
        new Translation2d(
            driveTranslation.getX(), MathUtil.clamp(driveTranslation.getY(), minDriveY, maxDriveY));
    var driveRotation =
        GeomUtil.translation3dTo2dXY(nodeTranslation)
            .minus(driveTranslation)
            .getAngle()
            .plus(Rotation2d.fromDegrees(shouldScoreFront() ? 0.0 : 180.0));
    return AllianceFlipUtil.apply(new Pose2d(driveTranslation, driveRotation));
  }

  /** Returns the best arm target for the selected node and drive position. */
  private ArmPose getArmTarget(Pose2d drivePosition) {
    var nodeTranslation = getNodeTranslation();
    drivePosition = AllianceFlipUtil.apply(drivePosition); // Unflipped

    // Calculate pose
    var distanceToNode =
        drivePosition.getTranslation().getDistance(GeomUtil.translation3dTo2dXY(nodeTranslation));
    var armPose =
        new ArmPose(
            new Translation2d(distanceToNode, nodeTranslation.getZ())
                .plus(getRelativeArmPosition()),
            getWristAngle());

    // Return pose with flip
    return armPose.withFlip(!shouldScoreFront());
  }

  /** Returns the position of the target node. */
  private Translation3d getNodeTranslation() {
    switch (objectiveTracker.selectedLevel) {
      case HYBRID:
        return FieldConstants.Grids.complexLow3dTranslations[objectiveTracker.selectedRow];
      case MID:
        return FieldConstants.Grids.mid3dTranslations[objectiveTracker.selectedRow];
      case HIGH:
        return FieldConstants.Grids.high3dTranslations[objectiveTracker.selectedRow];
      default:
        return new Translation3d();
    }
  }

  /** Returns the relative arm position for the selected node. */
  private Translation2d getRelativeArmPosition() {
    switch (objectiveTracker.selectedLevel) {
      case HYBRID:
        return hybridRelativePosition;
      case MID:
        switch (objectiveTracker.gamePiece) {
          case CUBE:
            return midCubeRelativePosition;
          case CONE:
            return midConeRelativePosition;
        }
        break;
      case HIGH:
        switch (objectiveTracker.gamePiece) {
          case CUBE:
            return highCubeRelativePosition;
          case CONE:
            return highConeRelativePosition;
        }
        break;
    }
    return new Translation2d();
  }

  /** Returns the wrist angle for the selected node. */
  private Rotation2d getWristAngle() {
    switch (objectiveTracker.selectedLevel) {
      case HYBRID:
        return hybridWristAngle;
      case MID:
        switch (objectiveTracker.gamePiece) {
          case CUBE:
            return midCubeWristAngle;
          case CONE:
            return midConeWristAngle;
        }
        break;
      case HIGH:
        switch (objectiveTracker.gamePiece) {
          case CUBE:
            return highCubeWristAngle;
          case CONE:
            return highConeWristAngle;
        }
        break;
    }
    return new Rotation2d();
  }
}
