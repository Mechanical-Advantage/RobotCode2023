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
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.List;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto(Drive drive, Arm arm, Gripper gripper) {
    var objective0 = new Objective(8, NodeLevel.HIGH, GamePiece.CONE, false);
    var objective1 = new Objective(7, NodeLevel.HIGH, GamePiece.CUBE, true);
    var objective2 = new Objective(6, NodeLevel.HIGH, GamePiece.CONE, true);

    Pose2d scorePosition0 =
        AutoScore.getDriveTarget(
            new Pose2d(
                FieldConstants.fieldLength / 2.0,
                FieldConstants.fieldWidth / 2.0,
                Rotation2d.fromDegrees(0.0)),
            objective0,
            arm,
            false);
    Translation2d midPoint =
        new Translation2d(
            (FieldConstants.Community.chargingStationInnerX
                    + FieldConstants.Community.chargingStationOuterX)
                / 2.0,
            (FieldConstants.Community.chargingStationLeftY + FieldConstants.Community.leftY) / 2.0);
    Pose2d intakePosition0 =
        new Pose2d(FieldConstants.StagingLocations.translations[3], new Rotation2d())
            .transformBy(new Transform2d(new Translation2d(-0.5, 0.0), new Rotation2d()));
    Pose2d scorePosition1 =
        AutoScore.getDriveTarget(
            new Pose2d(midPoint, Rotation2d.fromDegrees(0.0)), objective1, arm, true);
    Pose2d intakePosition1 =
        new Pose2d(FieldConstants.StagingLocations.translations[2], Rotation2d.fromDegrees(-30.0))
            .transformBy(new Transform2d(new Translation2d(-1.0, 0.0), new Rotation2d()));
    Pose2d scorePosition2 =
        AutoScore.getDriveTarget(
            new Pose2d(midPoint, Rotation2d.fromDegrees(0.0)), objective2, arm, true);
    Pose2d balancePosition0 =
        new Pose2d(
            FieldConstants.Community.chargingStationInnerX,
            FieldConstants.Community.chargingStationLeftY - 0.8,
            Rotation2d.fromDegrees(180.0));
    Pose2d balancePosition1 =
        new Pose2d(
            (FieldConstants.Community.chargingStationInnerX
                    + FieldConstants.Community.chargingStationOuterX)
                / 2.0,
            FieldConstants.Community.chargingStationLeftY - 0.8,
            Rotation2d.fromDegrees(180.0));

    addCommands(
        Commands.runOnce(() -> drive.setPose(scorePosition0)),
        arm.runPathCommand(AutoScore.getArmTarget(scorePosition0, objective0, arm, false)),
        gripper.ejectCommand(),
        new DriveTrajectory(
                drive,
                List.of(
                    Waypoint.fromHolonomicPose(scorePosition0),
                    new Waypoint(midPoint),
                    Waypoint.fromHolonomicPose(intakePosition0)))
            .alongWith(
                arm.runPathCommand(ArmPose.Preset.HOMED)
                    .andThen(
                        Commands.waitSeconds(0.25),
                        arm.runPathCommand(ArmPose.Preset.CUBE_HANDOFF))),
        new DriveTrajectory(
                drive,
                List.of(
                    Waypoint.fromHolonomicPose(intakePosition0),
                    Waypoint.fromDifferentialPose(
                        new Pose2d(midPoint, Rotation2d.fromDegrees(180.0))),
                    Waypoint.fromHolonomicPose(scorePosition1)))
            .alongWith(
                arm.runPathCommand(ArmPose.Preset.HOMED)
                    .andThen(
                        Commands.waitSeconds(0.25),
                        arm.runPathCommand(
                            AutoScore.getArmTarget(scorePosition1, objective1, arm, true)))),
        gripper.ejectCommand(),
        new DriveTrajectory(
                drive,
                List.of(
                    Waypoint.fromHolonomicPose(scorePosition1),
                    Waypoint.fromDifferentialPose(
                        new Pose2d(midPoint, Rotation2d.fromDegrees(0.0))),
                    Waypoint.fromHolonomicPose(intakePosition1)))
            .alongWith(
                arm.runPathCommand(ArmPose.Preset.HOMED)
                    .andThen(
                        Commands.waitSeconds(0.25),
                        arm.runPathCommand(ArmPose.Preset.FLOOR_CLOSE.getPose()))),
        new WaitCommand(0.75),
        new DriveTrajectory(
                drive,
                List.of(
                    Waypoint.fromHolonomicPose(intakePosition1),
                    Waypoint.fromDifferentialPose(
                        new Pose2d(midPoint, Rotation2d.fromDegrees(180.0))),
                    Waypoint.fromHolonomicPose(scorePosition2)))
            .alongWith(
                Commands.waitUntil(() -> AllianceFlipUtil.apply(drive.getRotation()).getCos() < 0.0)
                    .alongWith(arm.runPathCommand(ArmPose.Preset.HOMED))
                    .andThen(
                        arm.runPathCommand(
                            AutoScore.getArmTarget(scorePosition2, objective2, arm, true)))),
        gripper.ejectCommand(),
        Commands.runOnce(() -> arm.runPath(ArmPose.Preset.HOMED), arm),
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(scorePosition2),
                Waypoint.fromHolonomicPose(balancePosition0),
                Waypoint.fromHolonomicPose(balancePosition1)),
            List.of(new MaxVelocityConstraint(Units.feetToMeters(5.0)))));
  }
}
