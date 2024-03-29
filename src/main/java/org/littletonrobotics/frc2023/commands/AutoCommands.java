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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.FieldConstants.StagingLocations;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntake;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper.EjectSpeed;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.ConeOrientation;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;

@SuppressWarnings("unused")
public class AutoCommands {
  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Gripper gripper;
  private final CubeIntake cubeIntake;
  private final Supplier<List<AutoQuestionResponse>> responses;

  // Constants
  public static final boolean reachScoring = false;
  public static final double startX = Grids.outerX + 0.38;
  public static final double cubeIntakeDistance = 0.45;
  public static final double coneSweeperDistance = 0.1;
  public static final double coneSweeperBackoffDistance = 0.5;
  public static final double chargingStationMaxVelocity = Units.inchesToMeters(40.0);
  public static final double slowScoreConstraintRadius = 0.5;
  public static final double slowScoreMaxVelocity = Units.inchesToMeters(45.0);
  public static final Transform2d fieldCubeScoreTransform =
      new Transform2d(new Translation2d(0.18, 0.0), new Rotation2d());
  public static final Transform2d fieldSecondCubeIntakeTransform =
      new Transform2d(new Translation2d(0.15, -0.2), new Rotation2d());
  public static final Transform2d wallCubeScoreTransform =
      new Transform2d(new Translation2d(0.15, 0.0), new Rotation2d());
  public static final Transform2d wallFirstCubeIntakeTransform =
      new Transform2d(new Translation2d(0.5, 0.0), new Rotation2d());
  public static final Transform2d wallSecondCubeIntakeTransform =
      new Transform2d(new Translation2d(0.8, -0.2), new Rotation2d());

  // Waypoints
  public final Pose2d[] startingLocations = new Pose2d[9];
  public static final Rotation2d cableBumpRotationOut = Rotation2d.fromDegrees(30.0);
  public static final Rotation2d cableBumpRotationIn = Rotation2d.fromDegrees(-30.0);
  private final Translation2d transitWallSideNearOut;
  private final Translation2d transitWallSideNearIn;
  private final Translation2d transitWallSideCenterOut;
  private final Translation2d transitWallSideCenterIn;
  private final Translation2d transitWallSideFarOut;
  private final Translation2d transitWallSideFarIn;
  private final Waypoint transitWallSideNearOutWaypoint;
  private final Waypoint transitWallSideNearInWaypoint;
  private final Waypoint transitWallSideCenterOutWaypoint;
  private final Waypoint transitWallSideCenterInWaypoint;
  private final Waypoint transitWallSideFarOutWaypoint;
  private final Waypoint transitWallSideFarInWaypoint;
  private final Translation2d transitFieldSideNear;
  private final Translation2d transitFieldSideCenter;
  private final Translation2d transitFieldSideFar;
  private final Waypoint transitFieldSideNearOutWaypoint;
  private final Waypoint transitFieldSideNearInWaypoint;
  private final Waypoint transitFieldSideCenterOutWaypoint;
  private final Waypoint transitFieldSideCenterInWaypoint;
  private final Waypoint transitFieldSideFarOutWaypoint;
  private final Waypoint transitFieldSideFarInWaypoint;
  private final Translation2d chargingStationTransitNear;
  private final Translation2d chargingStationTransitFar;
  private final List<TrajectoryConstraint> trajectoryConstraints;

  private PIDController thetaController =
      new PIDController(6.0, 0.0, 0.0, Constants.loopPeriodSecs);

  public static record CommandWithPose(Command command, Pose2d pose) {}

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
      startingLocations[i] =
          new Pose2d(new Translation2d(startX, Grids.nodeY[i]), new Rotation2d());
    }
    transitWallSideNearOut =
        new Translation2d(
            Community.chargingStationInnerX + 0.5,
            (Community.chargingStationRightY + Community.rightY) / 2.0 - 0.15);
    transitWallSideNearIn =
        new Translation2d(
            Community.chargingStationInnerX + 0.5,
            (Community.chargingStationRightY + Community.rightY) / 2.0 + 0.15);
    transitWallSideCenterOut =
        new Translation2d(
            (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0,
            (Community.chargingStationRightY + Community.rightY) / 2.0 - 0.15);
    transitWallSideCenterIn =
        new Translation2d(
            (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0,
            (Community.chargingStationRightY + Community.rightY) / 2.0 + 0.15);
    transitWallSideFarOut =
        new Translation2d(
            Community.chargingStationOuterX - 0.5,
            (Community.chargingStationRightY + Community.rightY) / 2.0 - 0.15);
    transitWallSideFarIn =
        new Translation2d(
            Community.chargingStationOuterX - 0.5,
            (Community.chargingStationRightY + Community.rightY) / 2.0 + 0.15);
    transitWallSideNearOutWaypoint =
        new Waypoint(transitWallSideNearOut, new Rotation2d(), cableBumpRotationOut);
    transitWallSideNearInWaypoint =
        new Waypoint(transitWallSideNearIn, Rotation2d.fromDegrees(180.0), cableBumpRotationIn);
    transitWallSideCenterOutWaypoint =
        new Waypoint(transitWallSideCenterOut, new Rotation2d(), cableBumpRotationOut);
    transitWallSideCenterInWaypoint =
        new Waypoint(transitWallSideCenterIn, Rotation2d.fromDegrees(180.0), cableBumpRotationIn);
    transitWallSideFarOutWaypoint =
        new Waypoint(transitWallSideFarOut, new Rotation2d(), cableBumpRotationOut);
    transitWallSideFarInWaypoint =
        new Waypoint(transitWallSideFarIn, Rotation2d.fromDegrees(180.0), cableBumpRotationIn);
    transitFieldSideNear =
        new Translation2d(
            Community.chargingStationInnerX,
            (Community.chargingStationLeftY + Community.leftY) / 2.0);
    transitFieldSideCenter =
        new Translation2d(
            (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0,
            (Community.chargingStationLeftY + Community.leftY) / 2.0);
    transitFieldSideFar =
        new Translation2d(
            Community.chargingStationOuterX,
            (Community.chargingStationLeftY + Community.leftY) / 2.0);
    transitFieldSideNearInWaypoint =
        Waypoint.fromDifferentialPose(
            new Pose2d(transitFieldSideNear, Rotation2d.fromDegrees(180.0)));
    transitFieldSideNearOutWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitFieldSideNear, new Rotation2d()));
    transitFieldSideCenterInWaypoint =
        Waypoint.fromDifferentialPose(
            new Pose2d(transitFieldSideCenter, Rotation2d.fromDegrees(180.0)));
    transitFieldSideCenterOutWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitFieldSideCenter, new Rotation2d()));
    transitFieldSideFarInWaypoint =
        Waypoint.fromDifferentialPose(
            new Pose2d(transitFieldSideFar, Rotation2d.fromDegrees(180.0)));
    transitFieldSideFarOutWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitFieldSideFar, new Rotation2d()));
    chargingStationTransitNear =
        new Translation2d(
            Community.chargingStationInnerX,
            (Community.chargingStationLeftY + Community.chargingStationRightY) / 2.0);
    chargingStationTransitFar =
        new Translation2d(
            Community.chargingStationOuterX,
            (Community.chargingStationLeftY + Community.chargingStationRightY) / 2.0);
    trajectoryConstraints =
        List.of(
            // Charging station
            new RectangularRegionConstraint(
                new Translation2d(
                    Community.chargingStationInnerX - 0.8, Community.chargingStationRightY),
                new Translation2d(
                    Community.chargingStationOuterX + 0.8, Community.chargingStationLeftY),
                new MaxVelocityConstraint(chargingStationMaxVelocity)));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Reset the odometry to the specified pose. */
  private Command reset(Pose2d pose) {
    return runOnce(() -> drive.setPose(AllianceFlipUtil.apply(pose)));
  }

  /** Drives along the specified trajectory. */
  private Command path(List<Waypoint> waypoints) {
    return path(waypoints, List.of());
  }

  /** Drives along the specified trajectory. */
  private Command path(List<Waypoint> waypoints, List<TrajectoryConstraint> extraConstraints) {
    if (waypoints.size() == 2
        && waypoints.get(0).getDriveRotation().isEmpty()
        && waypoints.get(1).getDriveRotation().isEmpty()
        && waypoints.get(0).getTranslation().getDistance(waypoints.get(1).getTranslation()) < 0.5) {
      var driveToPose =
          new DriveToPose(
              drive,
              () ->
                  AllianceFlipUtil.apply(
                      new Pose2d(
                          waypoints.get(1).getTranslation(),
                          waypoints.get(1).getHolonomicRotation().get())));
      return driveToPose.until(driveToPose::atGoal);
    }
    List<TrajectoryConstraint> allConstraints = new ArrayList<>();
    allConstraints.addAll(trajectoryConstraints);
    allConstraints.addAll(extraConstraints);
    return new DriveTrajectory(drive, waypoints, allConstraints, 0.0);
  }

  /** Drives along the specified trajectory. */
  private Command path(Waypoint... waypoints) {
    return path(Arrays.asList(waypoints));
  }

  /** Runs to the homed arm position. */
  private Command armToHome() {
    return arm.runPathCommand(ArmPose.Preset.HOMED);
  }

  /** Drives to the specified game piece and grabs it. */
  private CommandWithPose driveAndIntake(
      Objective objective,
      boolean fieldSide,
      int intakePosition,
      Pose2d startingPose,
      boolean singleTransit) {
    return driveAndIntake(
        objective, fieldSide, intakePosition, startingPose, singleTransit, new Transform2d());
  }

  /** Drives to the specified game piece and grabs it. */
  private CommandWithPose driveAndIntake(
      Objective objective,
      boolean fieldSide,
      int intakePosition,
      Pose2d startingPose,
      boolean singleTransit,
      Transform2d intakeOffset) {
    // Create waypoints
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(Waypoint.fromHolonomicPose(startingPose));
    Waypoint transitWaypointNear =
        fieldSide ? transitFieldSideNearOutWaypoint : transitWallSideNearOutWaypoint;
    Waypoint transitWaypointCenter =
        fieldSide ? transitFieldSideCenterOutWaypoint : transitWallSideCenterOutWaypoint;
    Waypoint transitWaypointFar =
        fieldSide ? transitFieldSideFarOutWaypoint : transitWallSideFarOutWaypoint;
    if (singleTransit) {
      waypoints.add(transitWaypointCenter);
    } else {
      waypoints.add(transitWaypointNear);
      waypoints.add(transitWaypointFar);
    }

    Pose2d gamePiecePose =
        new Pose2d(
                StagingLocations.translations[intakePosition],
                StagingLocations.translations[intakePosition]
                    .minus(transitWaypointFar.getTranslation())
                    .getAngle())
            .transformBy(intakeOffset);
    if (objective.isConeNode()) {
      Waypoint lastWaypoint = waypoints.get(waypoints.size() - 1);
      waypoints.set(
          waypoints.size() - 1,
          new Waypoint(
              lastWaypoint.getTranslation(),
              lastWaypoint.getDriveRotation().get(),
              gamePiecePose.getRotation().plus(Rotation2d.fromDegrees(180.0))));
      waypoints.add(
          Waypoint.fromHolonomicPose(
              gamePiecePose.transformBy(
                  new Transform2d(
                      new Translation2d(-coneSweeperBackoffDistance, 0.0),
                      Rotation2d.fromDegrees(180.0))),
              gamePiecePose.getRotation()));
      waypoints.add(
          Waypoint.fromHolonomicPose(
              gamePiecePose.transformBy(
                  new Transform2d(
                      new Translation2d(-coneSweeperDistance, 0.0), Rotation2d.fromDegrees(180.0))),
              gamePiecePose.getRotation()));
    } else {
      waypoints.add(
          Waypoint.fromHolonomicPose(
              gamePiecePose.transformBy(translationToTransform(-cubeIntakeDistance, 0.0)),
              gamePiecePose.getRotation()));
    }

    // Create command
    return new CommandWithPose(
        path(waypoints)
            .alongWith(
                (objective.isConeNode()
                        ? arm.runPathCommand(ArmPose.Preset.HOMED)
                            .andThen(
                                Commands.waitUntil(
                                    () ->
                                        AllianceFlipUtil.apply(drive.getPose().getX())
                                            > FieldConstants.Community.chargingStationOuterX - 0.5))
                        : none())
                    .andThen(
                        arm.runPathCommand(
                            objective.isConeNode()
                                ? ArmPose.Preset.FLOOR_CONE
                                : ArmPose.Preset.CUBE_HANDOFF)))
            .deadlineWith(
                Commands.waitUntil(
                        () ->
                            AllianceFlipUtil.apply(drive.getPose().getX())
                                > FieldConstants.Community.chargingStationOuterX)
                    .deadlineWith(gripper.ejectCommand(EjectSpeed.FAST))
                    .andThen(gripper.intakeCommand()),
                (objective.isConeNode() ? none() : cubeIntake.runCommand())),
        new Pose2d(
            waypoints.get(waypoints.size() - 1).getTranslation(),
            waypoints.get(waypoints.size() - 1).getHolonomicRotation().get()));
  }

  /** Drives to the specified node and scores. */
  private CommandWithPose driveAndScore(
      Objective objective,
      boolean fieldSide,
      boolean firstScore,
      boolean slowAlign,
      Pose2d startingPose,
      boolean singleTransit) {
    return driveAndScore(
        objective,
        fieldSide,
        firstScore,
        slowAlign,
        startingPose,
        singleTransit,
        new Transform2d());
  }

  /** Drives to the specified node and scores. */
  private CommandWithPose driveAndScore(
      Objective objective,
      boolean fieldSide,
      boolean firstScore,
      boolean slowAlign,
      Pose2d startingPose,
      boolean singleTransit,
      Transform2d scoreOffset) {
    // Create waypoints
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(Waypoint.fromHolonomicPose(startingPose));
    Waypoint transitWaypointFar =
        fieldSide ? transitFieldSideFarInWaypoint : transitWallSideFarInWaypoint;
    Waypoint transitWaypointCenter =
        fieldSide ? transitFieldSideCenterInWaypoint : transitWallSideCenterInWaypoint;
    Waypoint transitWaypointNear =
        fieldSide ? transitFieldSideNearInWaypoint : transitWallSideNearInWaypoint;
    boolean includeTransit = startingPose.getX() > transitWaypointFar.getTranslation().getX();
    if (includeTransit) {
      if (singleTransit) {
        waypoints.add(transitWaypointCenter);
      } else {
        waypoints.add(transitWaypointFar);
        waypoints.add(transitWaypointNear);
      }
    }
    Pose2d scoringPoseUntransformed =
        AutoScore.getDriveTarget(
            new Pose2d(
                includeTransit
                    ? transitWaypointNear.getTranslation()
                    : startingPose.getTranslation(),
                startingPose.getRotation()),
            objective,
            arm,
            reachScoring);
    Pose2d scoringPose = scoringPoseUntransformed.transformBy(scoreOffset);
    waypoints.add(Waypoint.fromHolonomicPose(scoringPose));
    ArmPose scoringArmPose =
        AutoScore.getArmTarget(scoringPoseUntransformed, objective, arm, reachScoring);

    // Create command
    return new CommandWithPose(
        sequence(
            path(
                    waypoints,
                    slowAlign
                        ? List.of(
                            new EllipticalRegionConstraint(
                                scoringPose.getTranslation(),
                                slowScoreConstraintRadius * 2.0,
                                slowScoreConstraintRadius * 2.0,
                                new Rotation2d(),
                                new MaxVelocityConstraint(slowScoreMaxVelocity)))
                        : List.of())
                .alongWith(
                    (firstScore ? none() : gripper.intakeCommand().withTimeout(1.0)),
                    sequence(
                        firstScore ? none() : armToHome(),
                        waitUntil(
                            () ->
                                Math.abs(
                                        AllianceFlipUtil.apply(drive.getRotation())
                                            .minus(scoringPose.getRotation())
                                            .getDegrees())
                                    < 90.0),
                        arm.runPathCommand(scoringArmPose))),
            gripper.ejectCommand(objective)),
        scoringPose);
  }

  /** Drives to the charging station and balances on it. */
  private Command driveAndBalance(Pose2d startingPosition) {
    return driveAndBalance(startingPosition, true);
  }

  /** Drives to the charging station and balances on it. */
  public Command driveAndBalance(Pose2d startingPosition, boolean armToHome) {
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
            enterFront ? new Rotation2d() : Rotation2d.fromDegrees(180.0));
    Pose2d position1 =
        new Pose2d(
            (Community.chargingStationOuterX + Community.chargingStationInnerX) / 2.0
                + (enterFront ? 0.4 : -0.4),
            position0.getY(),
            position0.getRotation());
    return path(
            Waypoint.fromHolonomicPose(startingPosition),
            Waypoint.fromHolonomicPose(
                position0, enterFront ? new Rotation2d() : Rotation2d.fromDegrees(180.0)),
            Waypoint.fromHolonomicPose(position1))
        .alongWith(armToHome ? armToHome() : none())
        .andThen(new AutoBalance(drive));
  }

  /** Scores three game pieces on field-side (high cone, high cube, mid cube). */
  public Command fieldScoreThreeCombo() {
    var objective0 = new Objective(8, NodeLevel.HIGH, ConeOrientation.UPRIGHT, false);
    var objective1 = new Objective(7, NodeLevel.HIGH, ConeOrientation.UPRIGHT, false);
    var objective2 = new Objective(7, NodeLevel.MID, ConeOrientation.TIPPED, false);
    Pose2d startingPose = startingLocations[8];
    var score0Sequence = driveAndScore(objective0, true, true, false, startingPose, true);
    var intake0Sequence = driveAndIntake(objective1, true, 3, score0Sequence.pose(), true);
    var score1Sequence =
        driveAndScore(
            objective1, true, false, false, intake0Sequence.pose(), true, fieldCubeScoreTransform);
    var intake2Sequence =
        driveAndIntake(
            objective2, true, 2, score1Sequence.pose(), false, fieldSecondCubeIntakeTransform);
    var score2Sequence =
        driveAndScore(objective2, true, false, false, intake2Sequence.pose(), true);
    return sequence(
        reset(startingPose),
        score0Sequence.command(),
        intake0Sequence.command(),
        score1Sequence.command(),
        intake2Sequence.command(),
        score2Sequence.command());
  }

  /** Scores three game pieces on wall-side (high cone, high cube, mid cube). */
  public Command wallScoreThreeCombo() {
    var objective0 = new Objective(0, NodeLevel.HIGH, ConeOrientation.UPRIGHT, false);
    var objective1 = new Objective(1, NodeLevel.HIGH, ConeOrientation.UPRIGHT, false);
    var objective2 = new Objective(1, NodeLevel.MID, ConeOrientation.TIPPED, false);
    Pose2d startingPose = startingLocations[0];
    var score0Sequence = driveAndScore(objective0, false, true, false, startingPose, false);
    var intake0Sequence =
        driveAndIntake(
            objective1, false, 0, score0Sequence.pose(), true, wallFirstCubeIntakeTransform);
    var score1Sequence =
        driveAndScore(
            objective1, false, false, true, intake0Sequence.pose(), false, wallCubeScoreTransform);
    var intake2Sequence =
        driveAndIntake(
            objective2, false, 1, score1Sequence.pose(), false, wallSecondCubeIntakeTransform);
    var score2Sequence =
        driveAndScore(
            objective2, false, false, true, intake2Sequence.pose(), false, wallCubeScoreTransform);
    return sequence(
        reset(startingPose),
        score0Sequence.command(),
        intake0Sequence.command(),
        score1Sequence.command(),
        intake2Sequence.command(),
        score2Sequence.command());
  }

  /** Scores one cone and cube, then grabs and optionally balances. */
  public Command fieldScoreTwoGrabMaybeBalance() {
    Supplier<Boolean> balanceSupplier =
        () -> !responses.get().get(1).equals(AutoQuestionResponse.RETURN);
    Supplier<Boolean> scoreFinalSupplier =
        () -> responses.get().get(1).equals(AutoQuestionResponse.BALANCE_THROW);
    return select(
        Map.of(
            AutoQuestionResponse.HYBRID,
            sideScoreTwoMaybeGrabMaybeBalance(
                true, true, NodeLevel.HYBRID, balanceSupplier, scoreFinalSupplier),
            AutoQuestionResponse.MID,
            sideScoreTwoMaybeGrabMaybeBalance(
                true, true, NodeLevel.MID, balanceSupplier, scoreFinalSupplier),
            AutoQuestionResponse.HIGH,
            sideScoreTwoMaybeGrabMaybeBalance(
                true, true, NodeLevel.HIGH, balanceSupplier, scoreFinalSupplier)),
        () -> responses.get().get(0));
  }

  /** Scores one cone and cube, then optionally balance.s */
  public Command sideScoreTwoMaybeBalance() {
    Supplier<Boolean> balanceSupplier =
        () -> responses.get().get(2).equals(AutoQuestionResponse.YES);
    return either(
        select(
            Map.of(
                AutoQuestionResponse.HYBRID,
                sideScoreTwoMaybeGrabMaybeBalance(
                    true, false, NodeLevel.HYBRID, balanceSupplier, () -> false),
                AutoQuestionResponse.MID,
                sideScoreTwoMaybeGrabMaybeBalance(
                    true, false, NodeLevel.MID, balanceSupplier, () -> false),
                AutoQuestionResponse.HIGH,
                sideScoreTwoMaybeGrabMaybeBalance(
                    true, false, NodeLevel.HIGH, balanceSupplier, () -> false)),
            () -> responses.get().get(1)),
        select(
            Map.of(
                AutoQuestionResponse.HYBRID,
                sideScoreTwoMaybeGrabMaybeBalance(
                    false, false, NodeLevel.HYBRID, balanceSupplier, () -> false),
                AutoQuestionResponse.MID,
                sideScoreTwoMaybeGrabMaybeBalance(
                    false, false, NodeLevel.MID, balanceSupplier, () -> false),
                AutoQuestionResponse.HIGH,
                sideScoreTwoMaybeGrabMaybeBalance(
                    false, false, NodeLevel.HIGH, balanceSupplier, () -> false)),
            () -> responses.get().get(1)),
        () -> responses.get().get(0).equals(AutoQuestionResponse.FIELD_SIDE));
  }

  /** Scores one cone and cube, then optionally balance.s */
  private Command sideScoreTwoMaybeGrabMaybeBalance(
      boolean fieldSide,
      boolean grabThird,
      NodeLevel level,
      Supplier<Boolean> balanceSupplier,
      Supplier<Boolean> scoreFinalSupplier) {
    var objective0 = new Objective(fieldSide ? 8 : 0, level, ConeOrientation.UPRIGHT, false);
    var objective1 = new Objective(fieldSide ? 7 : 1, level, ConeOrientation.UPRIGHT, true);
    var objective2 = new Objective(fieldSide ? 7 : 1, level, ConeOrientation.UPRIGHT, true);
    Pose2d startingPose = startingLocations[fieldSide ? 8 : 0];
    var score0Sequence = driveAndScore(objective0, fieldSide, true, false, startingPose, false);
    var intake1Sequence =
        driveAndIntake(
            objective1,
            fieldSide,
            fieldSide ? 3 : 0,
            score0Sequence.pose(),
            false,
            fieldSide ? new Transform2d() : wallFirstCubeIntakeTransform);
    var score1Sequence =
        driveAndScore(
            objective1,
            fieldSide,
            false,
            !fieldSide,
            intake1Sequence.pose(),
            false,
            fieldSide ? fieldCubeScoreTransform : wallCubeScoreTransform);

    // Third grab sequences (field side only)
    var intake2Sequence =
        driveAndIntake(
            objective2,
            fieldSide,
            fieldSide ? 2 : 1,
            score1Sequence.pose(),
            false,
            fieldSecondCubeIntakeTransform);
    var returnWaypoints =
        List.of(
            Waypoint.fromHolonomicPose(intake2Sequence.pose()),
            fieldSide ? transitFieldSideFarInWaypoint : transitWallSideFarInWaypoint,
            new Waypoint(
                (fieldSide ? transitFieldSideNearInWaypoint : transitWallSideNearInWaypoint)
                    .getTranslation()
                    .plus(new Translation2d(-0.75, 0.0)),
                (fieldSide ? transitFieldSideNearInWaypoint : transitWallSideNearInWaypoint)
                    .getDriveRotation()
                    .get(),
                new Rotation2d()));

    return sequence(
        reset(startingPose),
        score0Sequence.command(),
        intake1Sequence.command(),
        score1Sequence.command(),
        grabThird
            ? sequence(
                intake2Sequence.command(),
                either(
                    driveAndBalance(intake2Sequence.pose(), false)
                        .alongWith(
                            gripper.intakeCommand(),
                            arm.runPathCommand(
                                () ->
                                    scoreFinalSupplier.get()
                                        ? ArmPose.Preset.THROW.getPose()
                                        : ArmPose.Preset.HOMED.getPose()))
                        .andThen(
                            either(
                                gripper.ejectCommand(EjectSpeed.VERY_FAST),
                                none(),
                                () -> scoreFinalSupplier.get())),
                    path(returnWaypoints).alongWith(armToHome()),
                    () -> balanceSupplier.get()))
            : either(
                driveAndBalance(score1Sequence.pose()), armToHome(), () -> balanceSupplier.get()));
  }

  /** Scores one game piece, gets mobility around the charge station, and optionally balances. */
  public Command sideScoreOneAndMaybeBalance() {
    Supplier<Boolean> balanceSupplier =
        () -> responses.get().get(3).equals(AutoQuestionResponse.YES);
    return either(
        select(
            Map.of(
                AutoQuestionResponse.HYBRID,
                select(
                    Map.of(
                        AutoQuestionResponse.WALL_SIDE,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.HYBRID, 6, balanceSupplier),
                        AutoQuestionResponse.CENTER,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.HYBRID, 7, balanceSupplier),
                        AutoQuestionResponse.FIELD_SIDE,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.HYBRID, 8, balanceSupplier)),
                    () -> responses.get().get(2)),
                AutoQuestionResponse.MID,
                select(
                    Map.of(
                        AutoQuestionResponse.WALL_SIDE,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.MID, 6, balanceSupplier),
                        AutoQuestionResponse.CENTER,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.MID, 7, balanceSupplier),
                        AutoQuestionResponse.FIELD_SIDE,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.MID, 8, balanceSupplier)),
                    () -> responses.get().get(2)),
                AutoQuestionResponse.HIGH,
                select(
                    Map.of(
                        AutoQuestionResponse.WALL_SIDE,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.HIGH, 6, balanceSupplier),
                        AutoQuestionResponse.CENTER,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.HIGH, 7, balanceSupplier),
                        AutoQuestionResponse.FIELD_SIDE,
                        sideScoreOneAndMaybeBalance(true, NodeLevel.HIGH, 8, balanceSupplier)),
                    () -> responses.get().get(2))),
            () -> responses.get().get(1)),
        select(
            Map.of(
                AutoQuestionResponse.HYBRID,
                select(
                    Map.of(
                        AutoQuestionResponse.WALL_SIDE,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.HYBRID, 0, balanceSupplier),
                        AutoQuestionResponse.CENTER,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.HYBRID, 1, balanceSupplier),
                        AutoQuestionResponse.FIELD_SIDE,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.HYBRID, 2, balanceSupplier)),
                    () -> responses.get().get(2)),
                AutoQuestionResponse.MID,
                select(
                    Map.of(
                        AutoQuestionResponse.WALL_SIDE,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.MID, 0, balanceSupplier),
                        AutoQuestionResponse.CENTER,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.MID, 1, balanceSupplier),
                        AutoQuestionResponse.FIELD_SIDE,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.MID, 2, balanceSupplier)),
                    () -> responses.get().get(2)),
                AutoQuestionResponse.HIGH,
                select(
                    Map.of(
                        AutoQuestionResponse.WALL_SIDE,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.HIGH, 0, balanceSupplier),
                        AutoQuestionResponse.CENTER,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.HIGH, 1, balanceSupplier),
                        AutoQuestionResponse.FIELD_SIDE,
                        sideScoreOneAndMaybeBalance(false, NodeLevel.HIGH, 2, balanceSupplier)),
                    () -> responses.get().get(2))),
            () -> responses.get().get(1)),
        () -> responses.get().get(0).equals(AutoQuestionResponse.FIELD_SIDE));
  }

  /** Scores one game piece, gets mobility around the charge station, and optionally balances. */
  private Command sideScoreOneAndMaybeBalance(
      boolean fieldSide, NodeLevel level, int position, Supplier<Boolean> balanceSupplier) {
    var objective = new Objective(position, level, ConeOrientation.UPRIGHT, false);
    var scoringSegment =
        driveAndScore(objective, false, true, false, startingLocations[position], false);
    var mobilityTransit =
        new Translation2d(
            Community.chargingStationOuterX + 1.0,
            fieldSide ? Community.chargingStationLeftY : Community.chargingStationRightY);
    var chargingStationPose =
        new Pose2d(
            Community.chargingStationOuterX,
            fieldSide
                ? Community.chargingStationLeftY - 0.8
                : Community.chargingStationRightY + 0.8,
            Rotation2d.fromDegrees(180.0));
    return sequence(
        reset(startingLocations[position]),
        scoringSegment.command(),
        either(
                path(
                        Waypoint.fromHolonomicPose(scoringSegment.pose()),
                        fieldSide
                            ? transitFieldSideNearOutWaypoint
                            : transitWallSideNearOutWaypoint,
                        fieldSide ? transitFieldSideFarOutWaypoint : transitWallSideFarOutWaypoint,
                        new Waypoint(mobilityTransit),
                        Waypoint.fromDifferentialPose(chargingStationPose, new Rotation2d()),
                        new Waypoint(
                            new Translation2d(
                                (Community.chargingStationOuterX + Community.chargingStationInnerX)
                                    / 2.0,
                                chargingStationPose.getY())))
                    .andThen(new AutoBalance(drive)),
                path(
                    Waypoint.fromHolonomicPose(scoringSegment.pose()),
                    fieldSide ? transitFieldSideNearOutWaypoint : transitWallSideNearOutWaypoint,
                    fieldSide ? transitFieldSideFarOutWaypoint : transitWallSideFarOutWaypoint,
                    new Waypoint(mobilityTransit)),
                () -> balanceSupplier.get())
            .alongWith(armToHome()));
  }

  /** Scores one game piece, gets mobility over the charge station, and balances */
  public Command centerScoreOneGrabAndBalance() {
    Supplier<Boolean> fieldSideSupplier =
        () -> responses.get().get(2).equals(AutoQuestionResponse.FIELD_SIDE);
    Supplier<Boolean> scoreFinalSupplier =
        () -> responses.get().get(3).equals(AutoQuestionResponse.YES);
    return select(
        Map.of(
            AutoQuestionResponse.HYBRID,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.HYBRID, 3, fieldSideSupplier, scoreFinalSupplier),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.HYBRID, 4, fieldSideSupplier, scoreFinalSupplier),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.HYBRID, 5, fieldSideSupplier, scoreFinalSupplier)),
                () -> responses.get().get(1)),
            AutoQuestionResponse.MID,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.MID, 3, fieldSideSupplier, scoreFinalSupplier),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.MID, 4, fieldSideSupplier, scoreFinalSupplier),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.MID, 5, fieldSideSupplier, scoreFinalSupplier)),
                () -> responses.get().get(1)),
            AutoQuestionResponse.HIGH,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.HIGH, 3, fieldSideSupplier, scoreFinalSupplier),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.HIGH, 4, fieldSideSupplier, scoreFinalSupplier),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneGrabAndBalance(
                        NodeLevel.HIGH, 5, fieldSideSupplier, scoreFinalSupplier)),
                () -> responses.get().get(1))),
        () -> responses.get().get(0));
  }

  /** Scores one game piece, gets mobility over the charge station, and balances */
  private Command centerScoreOneGrabAndBalance(
      NodeLevel level,
      int position,
      Supplier<Boolean> fieldSideSupplier,
      Supplier<Boolean> scoreFinalSupplier) {
    var objective = new Objective(position, level, ConeOrientation.UPRIGHT, false);
    var scoringSegment =
        driveAndScore(objective, false, true, false, startingLocations[position], false);
    var debouncerRising = new Debouncer(0.4, DebounceType.kRising);
    Pose2d intakePoseWall =
        new Pose2d(StagingLocations.translations[1], Rotation2d.fromDegrees(-15.0));
    Pose2d intakePoseField =
        new Pose2d(StagingLocations.translations[2], Rotation2d.fromDegrees(15.0));
    var driveToPose =
        new DriveToPose(
            drive,
            () ->
                AllianceFlipUtil.apply(fieldSideSupplier.get() ? intakePoseField : intakePoseWall));
    return sequence(
        reset(startingLocations[position]),
        scoringSegment.command(),
        path(
                Waypoint.fromHolonomicPose(scoringSegment.pose()),
                Waypoint.fromHolonomicPose(
                    new Pose2d(
                        chargingStationTransitNear.plus(new Translation2d(1.0, 0.0)),
                        new Rotation2d()),
                    new Rotation2d()))
            .alongWith(armToHome()),
        run(() ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        Units.inchesToMeters(
                            50.0 * (DriverStation.getAlliance() == Alliance.Red ? -1.0 : 1.0)),
                        0.0,
                        thetaController.calculate(
                            drive.getRotation().getRadians(),
                            AllianceFlipUtil.apply(new Rotation2d()).getRadians()),
                        drive.getRotation())))
            .until(() -> debouncerRising.calculate(Math.abs(drive.getPitch().getDegrees()) < 8.0)),
        gripper
            .intakeCommand()
            .withTimeout(2.0)
            .alongWith(arm.runPathCommand(ArmPose.Preset.CUBE_HANDOFF))
            .deadlineWith(driveToPose, cubeIntake.runCommand()),
        either(
                driveAndBalance(intakePoseField, false),
                driveAndBalance(intakePoseWall, false),
                () -> fieldSideSupplier.get())
            .alongWith(
                either(
                    arm.runPathCommand(() -> ArmPose.Preset.THROW.getPose())
                        .andThen(
                            waitUntil(
                                () ->
                                    AllianceFlipUtil.apply(drive.getPose().getX())
                                        < FieldConstants.Community.chargingStationOuterX - 0.5),
                            gripper.ejectCommand(EjectSpeed.VERY_FAST)),
                    armToHome(),
                    () -> scoreFinalSupplier.get())));
  }

  /** Scores one game piece, gets mobility over the charge station, and balances */
  public Command centerScoreOneMobilityAndBalance() {
    return select(
        Map.of(
            AutoQuestionResponse.HYBRID,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneMobilityAndBalance(NodeLevel.HYBRID, 3),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneMobilityAndBalance(NodeLevel.HYBRID, 4),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneMobilityAndBalance(NodeLevel.HYBRID, 5)),
                () -> responses.get().get(1)),
            AutoQuestionResponse.MID,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneMobilityAndBalance(NodeLevel.MID, 3),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneMobilityAndBalance(NodeLevel.MID, 4),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneMobilityAndBalance(NodeLevel.MID, 5)),
                () -> responses.get().get(1)),
            AutoQuestionResponse.HIGH,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneMobilityAndBalance(NodeLevel.HIGH, 3),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneMobilityAndBalance(NodeLevel.HIGH, 4),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneMobilityAndBalance(NodeLevel.HIGH, 5)),
                () -> responses.get().get(1))),
        () -> responses.get().get(0));
  }

  /** Scores one game piece, gets mobility over the charge station, and balances */
  private Command centerScoreOneMobilityAndBalance(NodeLevel level, int position) {
    var objective = new Objective(position, level, ConeOrientation.UPRIGHT, false);
    var scoringSegment =
        driveAndScore(objective, false, true, false, startingLocations[position], false);
    var debouncerRising = new Debouncer(0.4, DebounceType.kRising);
    var debouncerFalling = new Debouncer(0.4, DebounceType.kFalling);
    return sequence(
        reset(startingLocations[position]),
        scoringSegment.command(),
        path(
                Waypoint.fromHolonomicPose(scoringSegment.pose()),
                Waypoint.fromHolonomicPose(
                    new Pose2d(
                        chargingStationTransitNear.plus(new Translation2d(1.0, 0.0)),
                        new Rotation2d()),
                    new Rotation2d()))
            .alongWith(armToHome()),
        run(() ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        Units.inchesToMeters(
                            50.0 * (DriverStation.getAlliance() == Alliance.Red ? -1.0 : 1.0)),
                        0.0,
                        thetaController.calculate(
                            drive.getRotation().getRadians(),
                            AllianceFlipUtil.apply(new Rotation2d()).getRadians()),
                        drive.getRotation())))
            .until(() -> debouncerRising.calculate(Math.abs(drive.getPitch().getDegrees()) < 8.0)),
        run(() ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        Units.inchesToMeters(
                            -90.0 * (DriverStation.getAlliance() == Alliance.Red ? -1.0 : 1.0)),
                        0.0,
                        thetaController.calculate(
                            drive.getRotation().getRadians(),
                            AllianceFlipUtil.apply(new Rotation2d()).getRadians()),
                        drive.getRotation())))
            .until(
                () -> !debouncerFalling.calculate(Math.abs(drive.getPitch().getDegrees()) < 8.0)),
        new AutoBalance(drive));
  }

  /** Scores one game piece and balances from the center */
  public Command centerScoreOneAndBalance() {
    return select(
        Map.of(
            AutoQuestionResponse.HYBRID,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneAndBalance(NodeLevel.HYBRID, 3),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneAndBalance(NodeLevel.HYBRID, 4),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneAndBalance(NodeLevel.HYBRID, 5)),
                () -> responses.get().get(1)),
            AutoQuestionResponse.MID,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneAndBalance(NodeLevel.MID, 3),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneAndBalance(NodeLevel.MID, 4),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneAndBalance(NodeLevel.MID, 5)),
                () -> responses.get().get(1)),
            AutoQuestionResponse.HIGH,
            select(
                Map.of(
                    AutoQuestionResponse.WALL_SIDE,
                    centerScoreOneAndBalance(NodeLevel.HIGH, 3),
                    AutoQuestionResponse.CENTER,
                    centerScoreOneAndBalance(NodeLevel.HIGH, 4),
                    AutoQuestionResponse.FIELD_SIDE,
                    centerScoreOneAndBalance(NodeLevel.HIGH, 5)),
                () -> responses.get().get(1))),
        () -> responses.get().get(0));
  }

  /** Scores one game piece and balances from the center */
  private Command centerScoreOneAndBalance(NodeLevel level, int position) {
    var objective = new Objective(position, level, ConeOrientation.UPRIGHT, false);
    var scoringSegment =
        driveAndScore(objective, false, true, false, startingLocations[position], false);
    return sequence(
        reset(startingLocations[position]),
        scoringSegment.command(),
        armToHome(),
        Commands.waitUntil(arm::isTrajectoryFinished),
        driveAndBalance(scoringSegment.pose()));
  }
}
