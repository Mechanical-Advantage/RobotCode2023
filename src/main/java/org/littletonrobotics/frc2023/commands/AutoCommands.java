// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.littletonrobotics.frc2023.util.GeomUtil.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.FieldConstants.StagingLocations;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntake;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.ConeOrientation;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;

public class AutoCommands {
  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Gripper gripper;
  private final CubeIntake cubeIntake;
  private final Supplier<List<AutoQuestionResponse>> responses;

  // Constants
  private static final double startX = Grids.outerX + 0.5;
  private static final double throughHomeWaitSecs = 0.1;
  private static final double cubeIntakeDistance = 0.5;
  private static final double coneSweeperDistance = 1.0;
  private static final double coneSweeperBackoffDistance = 0.3;

  // Waypoints
  private final Pose2d[] startingForwards = new Pose2d[9];
  private final Pose2d[] startingBackwards = new Pose2d[9];
  private final Translation2d transitWallSide;
  private final Waypoint transitWallSideOutWaypoint;
  private final Waypoint transitWallSideInWaypoint;
  private final Translation2d transitFieldSide;
  private final Waypoint transitFieldSideOutWaypoint;
  private final Waypoint transitFieldSideInWaypoint;

  public AutoCommands(
      Drive drive,
      Arm arm,
      Gripper gripper,
      CubeIntake cubeIntake,
      Supplier<List<AutoQuestionResponse>> responses) {
    this.drive = drive;
    this.arm = arm;
    this.gripper = gripper;
    this.cubeIntake = cubeIntake;
    this.responses = responses;

    for (int i = 0; i < 9; i++) {
      startingForwards[i] =
          new Pose2d(new Translation2d(startX, Grids.nodeY[i]), Rotation2d.fromDegrees(180.0));
      startingBackwards[i] =
          new Pose2d(new Translation2d(startX, Grids.nodeY[i]), new Rotation2d());
    }
    transitWallSide =
        new Translation2d(
            (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0,
            (Community.chargingStationRightY + Community.rightY) / 2.0);
    transitWallSideInWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitWallSide, Rotation2d.fromDegrees(180.0)));
    transitWallSideOutWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitWallSide, new Rotation2d()));
    transitFieldSide =
        new Translation2d(
            (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0,
            (Community.chargingStationLeftY + Community.leftY) / 2.0);
    transitFieldSideInWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitFieldSide, Rotation2d.fromDegrees(180.0)));
    transitFieldSideOutWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitFieldSide, new Rotation2d()));
  }

  /** Reset the odometry to the specified pose. */
  private Command reset(Pose2d pose) {
    return runOnce(() -> drive.setPose(AllianceFlipUtil.apply(pose)));
  }

  /** Returns a waypoint for a holonomic pose. */
  private Waypoint holonomic(Pose2d pose) {
    return Waypoint.fromHolonomicPose(pose);
  }

  /** Drives along the specified trajectory. */
  private Command path(Waypoint... waypoints) {
    return new DriveTrajectory(drive, Arrays.asList(waypoints), List.of());
  }

  /** Drives along the specified trajectory. */
  private Command path(List<TrajectoryConstraint> constraints, Waypoint... waypoints) {
    return new DriveTrajectory(drive, Arrays.asList(waypoints), constraints);
  }

  /** Runs to the homed arm position. */
  private Command armToHome() {
    return arm.runPathCommand(ArmPose.Preset.HOMED);
  }

  /** Runs to the specified pose while stopping at homed. */
  private Command armPathThroughHome(ArmPose target) {
    return armToHome().andThen(waitSeconds(throughHomeWaitSecs), arm.runPathCommand(target));
  }

  /** Runs to the specified pose while stopping at homed. */
  private Command armPathThroughHome(ArmPose.Preset target) {
    return armPathThroughHome(target.getPose());
  }

  /** Drives to the charging station and balances on it. */
  private Command balance(Pose2d startingPosition) {
    boolean enterFront =
        startingPosition.getX()
            < (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0;
    Pose2d position0 =
        new Pose2d(
            enterFront ? Community.chargingStationInnerX : Community.chargingStationOuterX,
            MathUtil.clamp(
                startingPosition.getY(),
                Community.chargingStationRightY + 0.8,
                Community.chargingStationLeftY - 0.8),
            Rotation2d.fromDegrees(startingPosition.getRotation().getCos() > 0.0 ? 0.0 : 180.0));
    Pose2d position1 =
        new Pose2d(
            (Community.chargingStationOuterX + Community.chargingStationInnerX) / 2.0,
            position0.getY(),
            position0.getRotation());
    return path(
        Waypoint.fromHolonomicPose(startingPosition),
        Waypoint.fromHolonomicPose(
            position0, enterFront ? new Rotation2d() : Rotation2d.fromDegrees(180.0)),
        Waypoint.fromHolonomicPose(position1));
  }

  /** Places three game pieces on the selected level, then optionally balances. */
  public Command scoreLink() {
    Supplier<Boolean> balance = () -> responses.get().get(2) == AutoQuestionResponse.YES;
    return either(
        // Wall side
        select(
            Map.of(
                AutoQuestionResponse.HIGH,
                scoreLinkWithOptions(NodeLevel.HIGH, true, balance),
                AutoQuestionResponse.MID,
                scoreLinkWithOptions(NodeLevel.MID, true, balance),
                AutoQuestionResponse.HYBRID,
                scoreLinkWithOptions(NodeLevel.HYBRID, true, balance)),
            () -> responses.get().get(1)),

        // Field side
        select(
            Map.of(
                AutoQuestionResponse.HIGH,
                scoreLinkWithOptions(NodeLevel.HIGH, false, balance),
                AutoQuestionResponse.MID,
                scoreLinkWithOptions(NodeLevel.MID, false, balance),
                AutoQuestionResponse.HYBRID,
                scoreLinkWithOptions(NodeLevel.HYBRID, false, balance)),
            () -> responses.get().get(1)),
        () -> responses.get().get(0) == AutoQuestionResponse.WALL_SIDE);
  }

  /** Constructs a command to score a link with the specified options. */
  private Command scoreLinkWithOptions(
      NodeLevel level, boolean wallSide, Supplier<Boolean> balance) {
    var objective0 = new Objective(wallSide ? 0 : 8, level, ConeOrientation.TIPPED, false);
    var objective1 = new Objective(wallSide ? 1 : 7, level, ConeOrientation.TIPPED, true);
    var objective2 = new Objective(wallSide ? 2 : 6, level, ConeOrientation.TIPPED, true);

    var startingPose = startingBackwards[wallSide ? 0 : 8];
    var hybridBackupPose = startingPose.transformBy(translationToTransform(0.25, 0.0));
    var intakePose0 =
        new Pose2d(StagingLocations.translations[wallSide ? 0 : 3], new Rotation2d())
            .transformBy(translationToTransform(-cubeIntakeDistance, 0.0));
    var scorePose0 =
        AutoScore.getDriveTarget(
                new Pose2d(wallSide ? transitWallSide : transitFieldSide, new Rotation2d()),
                objective1,
                arm,
                true)
            .transformBy(
                translationToTransform(
                    level == NodeLevel.HYBRID ? -0.25 : (level == NodeLevel.MID ? -0.25 : 0.0),
                    0.0));
    var intakePose1 =
        new Pose2d(
                StagingLocations.translations[wallSide ? 1 : 2],
                Rotation2d.fromDegrees(wallSide ? 30.0 : -30.0))
            .transformBy(translationToTransform(-coneSweeperDistance, 0.0));
    var intakePose1Backoff =
        intakePose1.transformBy(translationToTransform(-coneSweeperBackoffDistance, 0.0));
    var scorePose1 =
        AutoScore.getDriveTarget(
                new Pose2d(wallSide ? transitWallSide : transitFieldSide, new Rotation2d()),
                objective2,
                arm,
                true)
            .transformBy(
                translationToTransform(
                    level == NodeLevel.HYBRID ? -0.25 : (level == NodeLevel.MID ? 0.25 : 0.0),
                    0.0));
    List<TrajectoryConstraint> constraints =
        List.of(
            new RectangularRegionConstraint(
                new Translation2d(Community.chargingStationInnerX, Community.rightY),
                new Translation2d(Community.chargingStationOuterX, Community.chargingStationRightY),
                new MaxVelocityConstraint(Units.inchesToMeters(80.0))));

    return sequence(
        reset(startingPose),
        arm.runPathCommand(
                AutoScore.getArmTarget(
                    level == NodeLevel.HYBRID ? hybridBackupPose : startingPose,
                    objective0,
                    arm,
                    false))
            .alongWith(
                level == NodeLevel.HYBRID
                    ? path(holonomic(startingPose), holonomic(hybridBackupPose))
                    : none()),
        gripper.ejectCommand(objective0),
        path(
                constraints,
                holonomic(level == NodeLevel.HYBRID ? hybridBackupPose : startingPose),
                wallSide ? transitWallSideOutWaypoint : transitFieldSideOutWaypoint,
                holonomic(intakePose0))
            .alongWith(
                armPathThroughHome(ArmPose.Preset.CUBE_HANDOFF)
                    .deadlineWith(parallel(cubeIntake.runCommand(), gripper.intakeCommand()))),
        path(
                constraints,
                holonomic(intakePose0),
                wallSide ? transitWallSideInWaypoint : transitFieldSideInWaypoint,
                holonomic(scorePose0))
            .alongWith(
                armPathThroughHome(AutoScore.getArmTarget(scorePose0, objective1, arm, true))),
        gripper.ejectCommand(objective1),
        path(
                constraints,
                holonomic(scorePose0),
                wallSide ? transitWallSideOutWaypoint : transitFieldSideOutWaypoint,
                holonomic(intakePose1Backoff))
            .alongWith(armPathThroughHome(ArmPose.Preset.FLOOR_CONE)),
        path(holonomic(intakePose1Backoff), holonomic(intakePose1))
            .deadlineWith(gripper.intakeCommand()),
        path(
                constraints,
                holonomic(intakePose1),
                wallSide ? transitWallSideInWaypoint : transitFieldSideInWaypoint,
                holonomic(scorePose1))
            .alongWith(
                waitUntil(
                        () ->
                            level == NodeLevel.HYBRID
                                || AllianceFlipUtil.apply(drive.getRotation()).getCos() < 0.0)
                    .alongWith(arm.runPathCommand(ArmPose.Preset.HOMED))
                    .andThen(
                        arm.runPathCommand(
                            AutoScore.getArmTarget(scorePose1, objective2, arm, true)))),
        gripper.ejectCommand(objective2),
        either(balance(scorePose1), none(), () -> balance.get()).alongWith(armToHome()));
  }
}
