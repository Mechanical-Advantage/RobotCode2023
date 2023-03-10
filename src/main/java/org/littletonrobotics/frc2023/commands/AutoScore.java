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
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.GeomUtil;

public class AutoScore extends SequentialCommandGroup {
  public static final double minDriveX = FieldConstants.Grids.outerX + 0.45;
  public static final double minDriveY = 0.7;
  public static final double maxDriveY = FieldConstants.Community.leftY - 0.5;
  public static final double minArmExtension = 0.8;
  public static final double maxArmExtensionHybrid = 0.9;
  public static final double maxArmExtensionMid = 1.1;
  public static final double maxArmExtensionHigh = 1.35;

  public static final double extendArmDriveTolerance = 1.0;
  public static final Rotation2d extendArmThetaTolerance = Rotation2d.fromDegrees(45.0);
  public static final double cubeHybridDriveTolerance = 0.1;
  public static final Rotation2d cubeHybridThetaTolerance = Rotation2d.fromDegrees(5.0);

  public static final Translation2d hybridRelativePosition = new Translation2d(-0.2, 0.4);
  public static final Rotation2d hybridWristAngle = Rotation2d.fromDegrees(-45.0);
  public static final Translation2d cubeRelativePosition = new Translation2d(-0.4, 0.5);
  public static final Rotation2d cubeWristAngle = Rotation2d.fromDegrees(-30.0);
  public static final Translation2d uprightConeRelativePosition = new Translation2d(-0.3, -0.025);
  public static final Rotation2d uprightConeWristAngle = Rotation2d.fromDegrees(55.0);
  public static final Translation2d tippedConeRelativePosition = new Translation2d(-0.29, -0.13);
  public static final Rotation2d tippedConeWristAngle = Rotation2d.fromDegrees(30.0);

  /** Auto score a game piece on the grid in full automatic mode. */
  public AutoScore(
      Drive drive,
      Arm arm,
      Gripper gripper,
      Objective objective,
      Supplier<Boolean> reachScoreDisable) {
    this(
        drive,
        arm,
        gripper,
        objective,
        Commands.none(),
        Commands.none(),
        () -> false,
        () -> false,
        () -> false,
        reachScoreDisable);
  }

  /** Auto score a game piece on the grid with support for overrides. */
  public AutoScore(
      Drive drive,
      Arm arm,
      Gripper gripper,
      Objective objective,
      Command driveWithJoysticks,
      Command moveArmWithJoysticks,
      Supplier<Boolean> ejectButton,
      Supplier<Boolean> fullManual,
      Supplier<Boolean> autoEject,
      Supplier<Boolean> reachScoreEnable) {

    // Create target suppliers
    Supplier<Pose2d> driveTargetSupplier =
        () ->
            AllianceFlipUtil.apply( // Flip
                getDriveTarget(
                    AllianceFlipUtil.apply(drive.getPose()), // Unflip
                    objective,
                    arm,
                    reachScoreEnable.get()));
    Supplier<ArmPose> armTargetSupplier =
        () ->
            getArmTarget(
                getDriveTarget(
                    AllianceFlipUtil.apply(drive.getPose()), // Unflip
                    objective,
                    arm,
                    reachScoreEnable.get()),
                objective,
                arm,
                reachScoreEnable.get());

    // Create drive and arm commands
    var driveToPose = new DriveToPose(drive, driveTargetSupplier);
    var driveCommand =
        driveToPose
            .until(
                () ->
                    fullManual.get()
                        ? true // Exit immediately and switch to joysticks in full manual
                        : (autoEject.get()
                            ? false // Never exit auto driving with auto eject
                            : atGoalForObjective( // Exit after initial alignment with manual eject
                                driveToPose, objective)))
            .andThen(driveWithJoysticks);
    var armCommand =
        Commands.waitUntil(
                () ->
                    fullManual.get()
                        || driveToPose.withinTolerance(
                            extendArmDriveTolerance, extendArmThetaTolerance))
            .andThen(
                arm.runPathCommand(armTargetSupplier),
                Commands.either(
                    Commands.run(() -> arm.runDirect(armTargetSupplier.get()), arm),
                    moveArmWithJoysticks,
                    () -> autoEject.get() && !fullManual.get()));

    // Combine all commands
    addCommands(
        Commands.either(
                Commands.waitUntil(() -> arm.isTrajectoryFinished() && driveToPose.atGoal()),
                Commands.waitUntil(() -> ejectButton.get()),
                () -> autoEject.get() && !fullManual.get())
            .deadlineWith(driveCommand, armCommand)
            .andThen(gripper.ejectCommand(objective))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }

  /**
   * Returns whether the drive to pose command is within the tolerance for the selected objective.
   */
  public static boolean atGoalForObjective(DriveToPose driveToPose, Objective objective) {
    if (objective.isConeNode()) {
      return driveToPose.atGoal();
    } else {
      return driveToPose.withinTolerance(cubeHybridDriveTolerance, cubeHybridThetaTolerance);
    }
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
    switch (objective.getScoringSide()) {
      case FRONT:
        return true;
      case BACK:
        return false;
      case EITHER:
        var nodeTranslation = GeomUtil.translation3dTo2dXY(getNodeTranslation(objective));
        var relativeRotation =
            nodeTranslation
                .minus(unflippedPose.getTranslation())
                .getAngle()
                .minus(unflippedPose.getRotation());
        return relativeRotation.getCos() > 0.0;
      default:
        return true;
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
      case HIGH:
        if (objective.isConeNode()) {
          switch (objective.coneOrientation) {
            case UPRIGHT:
              return uprightConeRelativePosition;
            case TIPPED:
              return tippedConeRelativePosition;
          }
        } else {
          return cubeRelativePosition;
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
      case HIGH:
        if (objective.isConeNode()) {
          switch (objective.coneOrientation) {
            case UPRIGHT:
              return uprightConeWristAngle;
            case TIPPED:
              return tippedConeWristAngle;
          }
        } else {
          return cubeWristAngle;
        }
        break;
    }
    return new Rotation2d();
  }
}
