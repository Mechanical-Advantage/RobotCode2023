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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.SuppliedWaitCommand;

public class AutoScore extends SequentialCommandGroup {
  public static final double minDriveX = FieldConstants.Grids.outerX + 0.45;
  public static final double minDriveY = 0.5;
  public static final double maxDriveY = FieldConstants.Community.leftY - 0.5;
  public static final double minArmExtension = 0.6;
  public static final double maxArmExtensionHybrid = 0.9;
  public static final double maxArmExtensionMid = 1.1;
  public static final double maxArmExtensionHigh = 1.35;

  public static final double extendArmDriveTolerance = 1.0;
  public static final Rotation2d extendArmThetaTolerance = Rotation2d.fromDegrees(45.0);
  public static final double cubeHybridDriveTolerance = 0.1;
  public static final Rotation2d cubeHybridThetaTolerance = Rotation2d.fromDegrees(5.0);
  public static final double coneMidScoreDelay = 1.0;
  public static final double coneHighScoreDelay = 1.5;

  public static final Translation2d hybridRelativePosition = new Translation2d(-0.2, 0.6);
  public static final Rotation2d hybridWristAngle = Rotation2d.fromDegrees(-60.0);
  public static final Translation2d midCubeRelativePosition = new Translation2d(-0.5, 0.5);
  public static final Rotation2d midCubeWristAngle = Rotation2d.fromDegrees(-30.0);
  public static final Translation2d highCubeRelativePosition = new Translation2d(-0.5, 0.5);
  public static final Rotation2d highCubeWristAngle = Rotation2d.fromDegrees(-30.0);
  public static final Translation2d midConeRelativePosition = new Translation2d(-0.165, 0.0);
  public static final Rotation2d midConeWristAngle = Rotation2d.fromDegrees(50.0);
  public static final Translation2d highConeRelativePosition = new Translation2d(-0.165, 0.0);
  public static final Rotation2d highConeWristAngle = Rotation2d.fromDegrees(50.0);

  private Supplier<Pose2d> driveTargetSupplier = null;
  private Supplier<ArmPose> armTargetSupplier = null;

  /** Auto score in full automatic mode with auto drive and auto arm. */
  public AutoScore(
      Drive drive,
      Arm arm,
      Gripper gripper,
      Objective objective,
      Supplier<Boolean> reachScoreDisable) {
    initTargetSuppliers(drive::getPose, arm, objective, reachScoreDisable);
    var driveCommand = new DriveToPose(drive, driveTargetSupplier);
    var armCommand =
        Commands.waitUntil(
                () ->
                    driveCommand.withinTolerance(extendArmDriveTolerance, extendArmThetaTolerance))
            .andThen(
                arm.runPathCommand(armTargetSupplier),
                Commands.run(() -> arm.runDirect(armTargetSupplier.get()), arm));
    addCommands(
        Commands.waitUntil(
                () -> atGoalForObjective(driveCommand, objective) && arm.isTrajectoryFinished())
            .andThen(waitToScore(objective))
            .deadlineWith(driveCommand, armCommand)
            .andThen(gripper.ejectCommand(objective))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }

  /** Auto score in semi-automatic mode with manual drive and auto arm. */
  public AutoScore(
      Supplier<Pose2d> poseSupplier,
      Arm arm,
      Gripper gripper,
      Objective objective,
      Supplier<Boolean> reachScoreDisable,
      Supplier<Boolean> ejectButton) {
    initTargetSuppliers(poseSupplier, arm, objective, reachScoreDisable);
    var armCommand =
        arm.runPathCommand(armTargetSupplier)
            .andThen(Commands.run(() -> arm.runDirect(armTargetSupplier.get()), arm));
    addCommands(
        Commands.waitUntil(() -> arm.isTrajectoryFinished() && ejectButton.get())
            .deadlineWith(armCommand)
            .andThen(gripper.ejectCommand(objective))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }

  /** Auto score in semi-automatic mode with auto drive and manual arm. */
  public AutoScore(
      Drive drive,
      Arm arm,
      Gripper gripper,
      Objective objective,
      Supplier<Boolean> reachScoreDisable,
      Supplier<Boolean> ejectButton,
      MoveArmWithJoysticks moveArmCommand) {
    initTargetSuppliers(drive::getPose, arm, objective, reachScoreDisable);
    var driveCommand = new DriveToPose(drive, driveTargetSupplier);
    var armCommand =
        Commands.waitUntil(
                () ->
                    driveCommand.withinTolerance(extendArmDriveTolerance, extendArmThetaTolerance))
            .andThen(arm.runPathCommand(armTargetSupplier), moveArmCommand);
    addCommands(
        Commands.waitUntil(
                () ->
                    atGoalForObjective(driveCommand, objective)
                        && arm.isTrajectoryFinished()
                        && ejectButton.get())
            .deadlineWith(driveCommand, armCommand)
            .andThen(gripper.ejectCommand(objective))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }

  /** Auto score in manual mode with manual drive and manual arm. */
  public AutoScore(
      Supplier<Pose2d> poseSupplier,
      Arm arm,
      Gripper gripper,
      Objective objective,
      Supplier<Boolean> reachScoreDisable,
      Supplier<Boolean> ejectButton,
      MoveArmWithJoysticks moveArmCommand) {
    initTargetSuppliers(poseSupplier, arm, objective, reachScoreDisable);
    var armCommand = arm.runPathCommand(armTargetSupplier).andThen(moveArmCommand);
    addCommands(
        Commands.waitUntil(() -> arm.isTrajectoryFinished() && ejectButton.get())
            .andThen(waitToScore(objective))
            .deadlineWith(armCommand)
            .andThen(gripper.ejectCommand(objective))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }

  /** Creates the drive and arm target suppliers (repeated in multiple constructors). */
  private void initTargetSuppliers(
      Supplier<Pose2d> poseSupplier,
      Arm arm,
      Objective objective,
      Supplier<Boolean> reachScoreDisable) {
    driveTargetSupplier =
        () ->
            AllianceFlipUtil.apply( // Flip
                getDriveTarget(
                    AllianceFlipUtil.apply(poseSupplier.get()), // Unflip
                    objective,
                    arm,
                    !reachScoreDisable.get()));
    armTargetSupplier =
        () ->
            getArmTarget(
                getDriveTarget(
                    AllianceFlipUtil.apply(poseSupplier.get()), // Unflip
                    objective,
                    arm,
                    !reachScoreDisable.get()),
                objective,
                arm,
                !reachScoreDisable.get());
  }

  /**
   * Returns whether the drive to pose command is within the tolerance for the selected objective.
   */
  public static boolean atGoalForObjective(DriveToPose driveToPose, Objective objective) {
    if (objective.nodeLevel == NodeLevel.HYBRID || objective.gamePiece == GamePiece.CUBE) {
      return driveToPose.withinTolerance(cubeHybridDriveTolerance, cubeHybridThetaTolerance);
    } else {
      return driveToPose.atGoal();
    }
  }

  /** Returns a command to wait after alignment based on the objective. */
  public static Command waitToScore(Objective objective) {
    return new SuppliedWaitCommand(
        () -> {
          if (objective.gamePiece == GamePiece.CONE) {
            switch (objective.nodeLevel) {
              case HYBRID:
                return 0.0;
              case MID:
                return coneMidScoreDelay;
              case HIGH:
                return coneHighScoreDelay;
            }
          }
          return 0.0;
        });
  }

  /** Returns the best drive target for the selected node. */
  public static Pose2d getDriveTarget(
      Pose2d unflippedPose, Objective objective, Arm arm, boolean allowReach) {
    var pose = unflippedPose;
    var nodeTranslation = getNodeTranslation(objective);
    var relativeArmPosition = getRelativeArmPosition(objective);

    // Select max arm extension
    double maxArmExtension = 0.0;
    switch (objective.nodeLevel) {
      case HYBRID -> maxArmExtension = maxArmExtensionHybrid;
      case MID -> maxArmExtension = maxArmExtensionMid;
      case HIGH -> maxArmExtension = maxArmExtensionHigh;
    }

    // Calculate drive distance
    double minDistance = minArmExtension - relativeArmPosition.getX();
    double maxDistance =
        Math.min(
                maxArmExtension,
                arm.calcMaxReachAtHeight(nodeTranslation.getZ() + relativeArmPosition.getY()))
            - relativeArmPosition.getX();
    double distanceFromNode =
        MathUtil.clamp(
            pose.getTranslation().getDistance(GeomUtil.translation3dTo2dXY(nodeTranslation)),
            minDistance,
            maxDistance);

    // If reach not allowed, return target at minimum distance
    if (!allowReach) {
      return new Pose2d(
          Math.max(minDriveX, nodeTranslation.getX() + minDistance),
          nodeTranslation.getY(),
          Rotation2d.fromDegrees(shouldScoreFront(pose, objective) ? 180.0 : 0.0));
    }

    // Calculate angle from node
    var angleFromNode =
        pose.getTranslation().minus(GeomUtil.translation3dTo2dXY(nodeTranslation)).getAngle();
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
            .plus(Rotation2d.fromDegrees(shouldScoreFront(pose, objective) ? 0.0 : 180.0));
    return new Pose2d(driveTranslation, driveRotation);
  }

  /** Returns the best arm target for the selected node and drive position. */
  public static ArmPose getArmTarget(
      Pose2d unflippedPose, Objective objective, Arm arm, boolean allowReach) {
    var nodeTranslation = getNodeTranslation(objective);

    // Calculate pose
    var distanceToNode =
        unflippedPose.getTranslation().getDistance(GeomUtil.translation3dTo2dXY(nodeTranslation));
    var armPose =
        new ArmPose(
            new Translation2d(distanceToNode, nodeTranslation.getZ())
                .plus(getRelativeArmPosition(objective)),
            getWristAngle(objective));

    // Return pose with flip
    return armPose.withFlip(!shouldScoreFront(unflippedPose, objective));
  }

  /** Returns whether to score off of the front or back of the robot. */
  public static boolean shouldScoreFront(Pose2d unflippedPose, Objective objective) {
    var nodeTranslation = GeomUtil.translation3dTo2dXY(getNodeTranslation(objective));
    var relativeRotation =
        nodeTranslation
            .minus(unflippedPose.getTranslation())
            .getAngle()
            .minus(unflippedPose.getRotation());
    if (objective.nodeLevel == NodeLevel.HYBRID || objective.gamePiece == GamePiece.CUBE) {
      // Choose nearest side
      return relativeRotation.getCos() > 0.0;

    } else {
      // Choose the same side as the cone was grabbed
      return objective.lastIntakeFront;
    }
  }

  /** Returns the position of the target node. */
  public static Translation3d getNodeTranslation(Objective objective) {
    switch (objective.nodeLevel) {
      case HYBRID:
        return FieldConstants.Grids.complexLow3dTranslations[objective.nodeRow];
      case MID:
        return FieldConstants.Grids.mid3dTranslations[objective.nodeRow];
      case HIGH:
        return FieldConstants.Grids.high3dTranslations[objective.nodeRow];
      default:
        return new Translation3d();
    }
  }

  /** Returns the relative arm position for the selected node. */
  public static Translation2d getRelativeArmPosition(Objective objective) {
    switch (objective.nodeLevel) {
      case HYBRID:
        return hybridRelativePosition;
      case MID:
        switch (objective.gamePiece) {
          case CUBE:
            return midCubeRelativePosition;
          case CONE:
            return midConeRelativePosition;
        }
        break;
      case HIGH:
        switch (objective.gamePiece) {
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
  public static Rotation2d getWristAngle(Objective objective) {
    switch (objective.nodeLevel) {
      case HYBRID:
        return hybridWristAngle;
      case MID:
        switch (objective.gamePiece) {
          case CUBE:
            return midCubeWristAngle;
          case CONE:
            return midConeWristAngle;
        }
        break;
      case HIGH:
        switch (objective.gamePiece) {
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
