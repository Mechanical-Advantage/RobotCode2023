// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;

public class TheUltimateAuto extends SequentialCommandGroup {
  private static final boolean reachScore = true;
  private static final Translation2d transit =
      new Translation2d(
          FieldConstants.Community.midX,
          (FieldConstants.Community.chargingStationLeftY + FieldConstants.Community.leftY) / 2.0);
  private static final Pose2d substationBackoff =
      DriveToSubstation.singleSubstationPose.transformBy(
          GeomUtil.translationToTransform(-0.8, 0.0));

  public TheUltimateAuto(Drive drive, Arm arm, Gripper gripper, ObjectiveTracker objectiveTracker) {
    List<Objective> objectives = getObjectiveList();
    for (int i = 0; i < objectives.size() - 1; i++) {
      addCommands(new AutoScore(drive, arm, gripper, objectives.get(i), () -> !reachScore));
      addCommands(Commands.waitUntil(() -> arm.isTrajectoryFinished()));
      addCommands(
          new DriveTrajectory(
              drive,
              () ->
                  List.of(
                      Waypoint.fromHolonomicPose(drive.getPose()),
                      Waypoint.fromDifferentialPose(new Pose2d(transit, new Rotation2d())),
                      Waypoint.fromHolonomicPose(substationBackoff))));
      addCommands(
          new DriveToSubstation(drive)
              .withTimeout(2.0)
              .andThen(
                  new DriveTrajectory(
                      drive,
                      List.of(
                          Waypoint.fromHolonomicPose(DriveToSubstation.singleSubstationPose),
                          Waypoint.fromHolonomicPose(substationBackoff))))
              .deadlineWith(
                  new IntakeSubstation(reachScore, arm, drive, gripper, new Objective())));
      Pose2d autoScorePose =
          AutoScore.getDriveTarget(
              new Pose2d(transit, Rotation2d.fromDegrees(180.0)),
              objectives.get(i + 1),
              arm,
              reachScore);
      addCommands(
          new DriveTrajectory(
              drive,
              () ->
                  List.of(
                      Waypoint.fromHolonomicPose(drive.getPose()),
                      Waypoint.fromDifferentialPose(
                          new Pose2d(transit, Rotation2d.fromDegrees(180.0))),
                      Waypoint.fromHolonomicPose(autoScorePose))));
    }
    addCommands(
        new AutoScore(
            drive, arm, gripper, objectives.get(objectives.size() - 1), () -> !reachScore));
    addCommands(
        new DriveTrajectory(
            drive,
            () ->
                List.of(
                    Waypoint.fromHolonomicPose(drive.getPose()),
                    Waypoint.fromDifferentialPose(new Pose2d(transit, new Rotation2d())),
                    Waypoint.fromHolonomicPose(
                        new Pose2d(
                            FieldConstants.fieldLength / 2.0,
                            FieldConstants.fieldWidth / 2.0,
                            new Rotation2d())))));
    addCommands(
        Commands.run(
                () ->
                    drive.runVelocity(
                        new ChassisSpeeds(0.0, 0.0, drive.getMaxAngularSpeedRadPerSec() * 0.75)))
            .withTimeout(5.0));
  }

  private static List<Objective> getObjectiveList() {
    List<Objective> objectives = new ArrayList<>();
    objectives.add(new Objective(0, NodeLevel.HYBRID, GamePiece.CUBE, true));
    objectives.add(new Objective(1, NodeLevel.HYBRID, GamePiece.CONE, true));
    objectives.add(new Objective(2, NodeLevel.HYBRID, GamePiece.CUBE, true));
    objectives.add(new Objective(3, NodeLevel.HYBRID, GamePiece.CUBE, true));
    objectives.add(new Objective(4, NodeLevel.HYBRID, GamePiece.CONE, true));
    objectives.add(new Objective(5, NodeLevel.HYBRID, GamePiece.CUBE, true));
    objectives.add(new Objective(6, NodeLevel.HYBRID, GamePiece.CUBE, true));
    objectives.add(new Objective(7, NodeLevel.HYBRID, GamePiece.CONE, true));
    objectives.add(new Objective(8, NodeLevel.HYBRID, GamePiece.CUBE, true));
    objectives.add(new Objective(0, NodeLevel.MID, GamePiece.CONE, true));
    objectives.add(new Objective(1, NodeLevel.MID, GamePiece.CUBE, true));
    objectives.add(new Objective(2, NodeLevel.MID, GamePiece.CONE, true));
    objectives.add(new Objective(3, NodeLevel.MID, GamePiece.CONE, true));
    objectives.add(new Objective(4, NodeLevel.MID, GamePiece.CUBE, true));
    objectives.add(new Objective(5, NodeLevel.MID, GamePiece.CONE, true));
    objectives.add(new Objective(6, NodeLevel.MID, GamePiece.CONE, true));
    objectives.add(new Objective(7, NodeLevel.MID, GamePiece.CUBE, true));
    objectives.add(new Objective(8, NodeLevel.MID, GamePiece.CONE, true));
    objectives.add(new Objective(0, NodeLevel.HIGH, GamePiece.CONE, true));
    objectives.add(new Objective(1, NodeLevel.HIGH, GamePiece.CUBE, true));
    objectives.add(new Objective(2, NodeLevel.HIGH, GamePiece.CONE, true));
    objectives.add(new Objective(3, NodeLevel.HIGH, GamePiece.CONE, true));
    objectives.add(new Objective(4, NodeLevel.HIGH, GamePiece.CUBE, true));
    objectives.add(new Objective(5, NodeLevel.HIGH, GamePiece.CONE, true));
    objectives.add(new Objective(6, NodeLevel.HIGH, GamePiece.CONE, true));
    objectives.add(new Objective(7, NodeLevel.HIGH, GamePiece.CUBE, true));
    objectives.add(new Objective(8, NodeLevel.HIGH, GamePiece.CONE, true));
    return objectives;
  }
}
