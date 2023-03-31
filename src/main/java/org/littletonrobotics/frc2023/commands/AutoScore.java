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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;

public class AutoScore extends SequentialCommandGroup {
  public static final double bendCompensation = 0.0;
  // Units.inchesToMeters(-1.0); // Blue alliance
  // Units.inchesToMeters(1.0); // Red alliance

  public static Transform2d getBendCompensation(boolean isFront) {
    return new Transform2d(
        new Translation2d(0.0, bendCompensation * (isFront ? 1.0 : -1.0)), new Rotation2d());
  }

  public static final double minDriveX = FieldConstants.Grids.outerX + 0.45;
  public static final double minDriveY = 0.7;
  public static final double maxDriveY = FieldConstants.Community.leftY - 0.5;
  public static final double minArmExtension = 0.3;
  public static final double maxArmExtensionHybrid = 0.4;
  public static final double maxArmExtensionMid = 1.0;
  public static final double maxArmExtensionHigh = 1.35;

  public static final double extendArmDriveTolerance = 3.0;
  public static final Rotation2d extendArmThetaTolerance = Rotation2d.fromDegrees(45.0);
  public static final Rotation2d extendArmTippingTolerancePosition = Rotation2d.fromDegrees(2.5);
  public static final Rotation2d extendArmTippingToleranceVelocity = Rotation2d.fromDegrees(5.0);
  public static final double cubeHybridDriveTolerance = 0.1;
  public static final Rotation2d cubeHybridThetaTolerance = Rotation2d.fromDegrees(5.0);

  public static final Translation2d hybridRelativePosition = new Translation2d(-0.6, 0.5);
  public static final Rotation2d hybridWristAngle = new Rotation2d();
  public static final Translation2d cubeMidRelativePosition = new Translation2d(-0.4, 0.5);
  public static final Rotation2d cubeMidWristAngle = Rotation2d.fromDegrees(-30.0);
  public static final Translation2d cubeHighRelativePosition = new Translation2d(-0.5, 0.2);
  public static final Rotation2d cubeHighWristAngle = Rotation2d.fromDegrees(45.0);
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
    var trajectoryCommand =
        Commands.either(
            makeTrajectoryCommand(drive, () -> AllianceFlipUtil.apply(driveTargetSupplier.get())),
            new DriveToPose(drive, driveTargetSupplier),
            () ->
                drive
                        .getPose()
                        .getTranslation()
                        .getDistance(driveTargetSupplier.get().getTranslation())
                    > 0.5);
    var driveCommand =
        trajectoryCommand
            .until(
                () ->
                    fullManual.get()
                        ? true // Exit immediately and switch to joysticks in full manual
                        : (autoEject.get()
                            ? false // Never exit auto driving with auto eject
                            : atGoalForObjective( // Exit after initial alignment with manual eject
                                drive.getPose(), driveTargetSupplier.get(), objective)))
            .andThen(driveWithJoysticks);
    var armCommand =
        Commands.either(
                Commands.none(),
                Commands.waitSeconds(0.2) // Wait minimum time in case robot is about to tip
                    .andThen(
                        Commands.waitUntil(
                            () -> {
                              Pose2d relativePose =
                                  drive.getPose().relativeTo(driveTargetSupplier.get());
                              return relativePose.getTranslation().getNorm()
                                      < extendArmDriveTolerance
                                  && Math.abs(relativePose.getRotation().getRadians())
                                      < extendArmThetaTolerance.getRadians()
                                  && Math.abs(drive.getPitch().getRadians())
                                      < extendArmTippingTolerancePosition.getRadians()
                                  && Math.abs(drive.getRoll().getRadians())
                                      < extendArmTippingTolerancePosition.getRadians()
                                  && Math.abs(drive.getPitchVelocity())
                                      < extendArmTippingToleranceVelocity.getRadians()
                                  && Math.abs(drive.getRollVelocity())
                                      < extendArmTippingToleranceVelocity.getRadians();
                            })),
                () -> fullManual.get())
            .andThen(
                arm.runPathCommand(armTargetSupplier),
                Commands.either(
                    Commands.run(() -> arm.runDirect(armTargetSupplier.get()), arm),
                    moveArmWithJoysticks,
                    () -> autoEject.get() && !fullManual.get()));

    // Combine all commands
    addCommands(
        Commands.either(
                Commands.waitUntil(
                    () -> arm.isTrajectoryFinished() && trajectoryCommand.isFinished()),
                Commands.waitUntil(() -> ejectButton.get()),
                () -> autoEject.get() && !fullManual.get())
            .deadlineWith(driveCommand, armCommand)
            .andThen(gripper.ejectCommand(objective))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }

  /**
   * Returns whether the drive to pose command is within the tolerance for the selected objective.
   */
  public static boolean atGoalForObjective(Pose2d pose, Pose2d target, Objective objective) {
    if (objective.isConeNode()) {
      return false; // Allow trajectory command to exit on its own
    } else {
      Pose2d relativePose = pose.relativeTo(target);
      return relativePose.getTranslation().getNorm() < cubeHybridDriveTolerance
          && Math.abs(relativePose.getRotation().getRadians())
              < cubeHybridThetaTolerance.getRadians();
    }
  }

  /** Returns a command to drive to the supplied target. */
  public static Command makeTrajectoryCommand(Drive drive, Supplier<Pose2d> targetSupplier) {
    Supplier<Pose2d> unflippedPoseSupplier = () -> AllianceFlipUtil.apply(drive.getPose());
    Supplier<Double> unflippedFieldVelocityXSupplier =
        () ->
            DriverStation.getAlliance() == Alliance.Red
                ? -drive.getFieldVelocity().dx
                : drive.getFieldVelocity().dx;
    Supplier<Boolean> movingStart =
        () ->
            new Translation2d(unflippedFieldVelocityXSupplier.get(), drive.getFieldVelocity().dy)
                        .getNorm()
                    > drive.getMaxLinearSpeedMetersPerSec() * 0.5
                && unflippedPoseSupplier.get().getX() > FieldConstants.Community.midX;
    return new DriveTrajectory(
        drive,
        () -> {
          List<Waypoint> waypoints = new ArrayList<>();
          Pose2d startPose = unflippedPoseSupplier.get();
          Pose2d targetPose = targetSupplier.get();
          waypoints.add(
              Waypoint.fromHolonomicPose(
                  startPose,
                  movingStart.get()
                      ? new Rotation2d(
                          unflippedFieldVelocityXSupplier.get(), drive.getFieldVelocity().dy)
                      : null));

          // Enter through field side passage
          boolean addFieldSideInner =
              startPose.getX() > FieldConstants.Community.chargingStationInnerX
                  && startPose.getY() > FieldConstants.Community.chargingStationLeftY;
          if (startPose.getX() > FieldConstants.Community.outerX
              && startPose.getY()
                  > (FieldConstants.Community.chargingStationLeftY
                          + FieldConstants.Community.chargingStationRightY)
                      / 2.0
              && startPose.getY() < FieldConstants.Community.chargingStationLeftY) {
            addFieldSideInner = true;
            waypoints.add(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationOuterX - 0.4,
                        FieldConstants.Community.chargingStationLeftY + 0.5),
                    Rotation2d.fromDegrees(180.0),
                    targetPose.getRotation()));
          }
          if (addFieldSideInner) {
            waypoints.add(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationInnerX - 0.2,
                        MathUtil.clamp(
                            targetPose.getY(),
                            FieldConstants.Community.chargingStationLeftY + 0.5,
                            FieldConstants.Community.leftY - 0.5)),
                    Rotation2d.fromDegrees(180.0),
                    targetPose.getRotation()));
          }

          // Enter through wall side passage
          boolean addWallSideInner =
              startPose.getX() > FieldConstants.Community.chargingStationInnerX
                  && startPose.getY() < FieldConstants.Community.chargingStationRightY;
          boolean wallSideInnerSetHolonomic = false;
          if (startPose.getX() > FieldConstants.Community.outerX
              && startPose.getY()
                  < (FieldConstants.Community.chargingStationLeftY
                          + FieldConstants.Community.chargingStationRightY)
                      / 2.0) {
            addWallSideInner = true;
            wallSideInnerSetHolonomic = true;
            waypoints.add(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationOuterX - 0.4,
                        MathUtil.clamp(
                            targetPose.getY(),
                            FieldConstants.Community.rightY + 0.5,
                            FieldConstants.Community.chargingStationRightY - 0.5)),
                    Rotation2d.fromDegrees(180.0),
                    targetPose.getRotation()));
          }
          if (addWallSideInner) {
            waypoints.add(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationInnerX - 0.2,
                        MathUtil.clamp(
                            targetPose.getY(),
                            FieldConstants.Community.rightY + 0.5,
                            FieldConstants.Community.chargingStationRightY - 0.5)),
                    Rotation2d.fromDegrees(180.0),
                    wallSideInnerSetHolonomic
                        ? targetPose.getRotation()
                        : startPose.getRotation()));
          }

          // Enter through charge station
          if (startPose.getX() > FieldConstants.Community.chargingStationInnerX
              && startPose.getX() < FieldConstants.Community.chargingStationOuterX
              && startPose.getY() > FieldConstants.Community.chargingStationRightY
              && startPose.getY() < FieldConstants.Community.chargingStationLeftY) {
            waypoints.add(
                new Waypoint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationInnerX - 0.2, startPose.getY()),
                    Rotation2d.fromDegrees(180.0),
                    startPose.getRotation()));
          }

          // Avoid charge station corners
          Translation2d lastTranslation = waypoints.get(waypoints.size() - 1).getTranslation();
          if (lastTranslation.getX()
                  > (FieldConstants.Community.midX + FieldConstants.Grids.outerX) / 2.0
              && lastTranslation.getX() < FieldConstants.Community.midX) {
            if (lastTranslation.getY() > FieldConstants.Community.chargingStationLeftY
                && lastTranslation.getY() < FieldConstants.Community.leftY
                && targetPose.getY() < FieldConstants.Community.chargingStationLeftY - 0.8) {
              // Left corner
              var translation =
                  new Translation2d(
                      (FieldConstants.Community.midX + FieldConstants.Grids.outerX) / 2.0,
                      FieldConstants.Community.chargingStationLeftY - 0.2);
              waypoints.add(
                  Waypoint.fromDifferentialPose(
                      new Pose2d(
                          translation, targetPose.getTranslation().minus(translation).getAngle())));
            } else if (lastTranslation.getY() < FieldConstants.Community.chargingStationRightY
                && lastTranslation.getY() > FieldConstants.Community.rightY
                && targetPose.getY() > FieldConstants.Community.chargingStationRightY + 0.8) {
              // Right corner
              var translation =
                  new Translation2d(
                      (FieldConstants.Community.midX + FieldConstants.Grids.outerX) / 2.0,
                      FieldConstants.Community.chargingStationRightY + 0.2);
              waypoints.add(
                  Waypoint.fromDifferentialPose(
                      new Pose2d(
                          translation, targetPose.getTranslation().minus(translation).getAngle())));
            }
          }

          waypoints.add(Waypoint.fromHolonomicPose(targetPose));
          return waypoints;
        },
        () ->
            List.of(
                // Ending position
                new EllipticalRegionConstraint(
                    targetSupplier.get().getTranslation(),
                    AutoCommands.slowScoreConstraintRadius * 2.0,
                    AutoCommands.slowScoreConstraintRadius * 2.0,
                    new Rotation2d(),
                    new MaxVelocityConstraint(AutoCommands.slowScoreMaxVelocity)),

                // Cable bump
                new RectangularRegionConstraint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationInnerX,
                        FieldConstants.Community.rightY),
                    new Translation2d(
                        FieldConstants.Community.chargingStationOuterX,
                        FieldConstants.Community.chargingStationRightY),
                    new MaxVelocityConstraint(AutoCommands.cableBumpMaxVelocity)),

                // Charging station
                new RectangularRegionConstraint(
                    new Translation2d(
                        FieldConstants.Community.chargingStationInnerX,
                        FieldConstants.Community.chargingStationRightY),
                    new Translation2d(
                        FieldConstants.Community.chargingStationOuterX,
                        FieldConstants.Community.chargingStationLeftY),
                    new MaxVelocityConstraint(AutoCommands.chargingStationMaxVelocity))),
        () ->
            movingStart.get()
                ? new Translation2d(
                        unflippedFieldVelocityXSupplier.get(), drive.getFieldVelocity().dy)
                    .getNorm()
                : 0.0);
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
      boolean front = shouldScoreFront(pose, objective);
      return new Pose2d(
              Math.max(minDriveX, nodeTranslation.getX() + minDistance),
              nodeTranslation.getY(),
              Rotation2d.fromDegrees(front ? 180.0 : 0.0))
          .transformBy(getBendCompensation(front));
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
    return new Pose2d(driveTranslation, driveRotation)
        .transformBy(getBendCompensation(shouldScoreFront(pose, objective)));
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
    if (objective.isConeNode()) {
      switch (objective.nodeLevel) {
        case HYBRID:
          return hybridRelativePosition;
        case MID:
        case HIGH:
          switch (objective.coneOrientation) {
            case UPRIGHT:
              return uprightConeRelativePosition;
            case TIPPED:
              return tippedConeRelativePosition;
          }
          break;
      }
    } else {
      switch (objective.nodeLevel) {
        case HYBRID:
          return hybridRelativePosition;
        case MID:
          return cubeMidRelativePosition;
        case HIGH:
          return cubeHighRelativePosition;
      }
    }
    return new Translation2d();
  }

  /** Returns the wrist angle for the selected node. */
  public static Rotation2d getWristAngle(Objective objective) {
    if (objective.isConeNode()) {
      switch (objective.nodeLevel) {
        case HYBRID:
          return hybridWristAngle;
        case MID:
        case HIGH:
          switch (objective.coneOrientation) {
            case UPRIGHT:
              return uprightConeWristAngle;
            case TIPPED:
              return tippedConeWristAngle;
          }
          break;
      }
    } else {
      switch (objective.nodeLevel) {
        case HYBRID:
          return hybridWristAngle;
        case MID:
          return cubeMidWristAngle;
        case HIGH:
          return cubeHighWristAngle;
      }
    }
    return new Rotation2d();
  }
}
