// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.commands.DriveTrajectory;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntake;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FieldSideThreePiece extends SequentialCommandGroup {

  private static final Pose2d startPosition =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .5, FieldConstants.Grids.nodeY[7]),
          new Rotation2d(Math.PI));

  private static final Pose2d exitIntermediatePosition =
      new Pose2d(
          new Translation2d(
              FieldConstants.Community.chargingStationOuterX,
              ((FieldConstants.Community.chargingStationLeftY + FieldConstants.Community.leftY) / 2)
                  + .09),
          new Rotation2d());

  private static final Pose2d secondPiece =
      new Pose2d(
          new Translation2d(
              FieldConstants.StagingLocations.positionX - .28,
              FieldConstants.StagingLocations.firstY
                  + (FieldConstants.StagingLocations.separationY * 3)
                  - .43),
          Rotation2d.fromDegrees(50));

  private static final Pose2d secondPieceIntermediate =
      new Pose2d(
          new Translation2d(
              FieldConstants.Community.chargingStationInnerX,
              ((FieldConstants.Community.chargingStationLeftY + FieldConstants.Community.leftY) / 2)
                  + .15),
          new Rotation2d(Math.PI));

  private static final Pose2d secondPieceScore =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .63, FieldConstants.Grids.nodeY[8] - .12),
          Rotation2d.fromDegrees(160));

  private static final Pose2d thirdPieceIntermediate =
      new Pose2d(
          new Translation2d(
              FieldConstants.Community.chargingStationOuterX,
              ((FieldConstants.Community.chargingStationLeftY + FieldConstants.Community.leftY) / 2)
                  + .09),
          new Rotation2d());
  private static final Pose2d thirdPieceReturnIntermediateIntermediate =
      new Pose2d(
          new Translation2d(
              FieldConstants.StagingLocations.positionX - 1,
              FieldConstants.StagingLocations.firstY
                  + (FieldConstants.StagingLocations.separationY * 3)),
          new Rotation2d(Math.PI));

  private static final Pose2d thirdPieceReturnIntermediate =
      new Pose2d(
          new Translation2d(
              FieldConstants.Community.chargingStationOuterX - 1.55,
              ((FieldConstants.Community.chargingStationLeftY + FieldConstants.Community.leftY) / 2)
                  + .08),
          new Rotation2d(Math.PI / 3));

  private static final Pose2d thirdPiece =
      new Pose2d(
          new Translation2d(
              FieldConstants.StagingLocations.positionX - .30,
              FieldConstants.StagingLocations.firstY
                  + (FieldConstants.StagingLocations.separationY * 2)
                  - .40),
          Rotation2d.fromDegrees(40));

  private static final Pose2d thirdPieceScore =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .6, FieldConstants.Grids.nodeY[6]),
          new Rotation2d(Math.PI));

  /** Creates a new WallsideThreePiece. */
  public FieldSideThreePiece(Drive drive, CubeIntake cubeIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(startPosition))));

    addCommands(cubeIntake.ejectCommand().withTimeout(.5));

    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(startPosition),
                Waypoint.fromDifferentialPose(exitIntermediatePosition),
                Waypoint.fromHolonomicPose(secondPiece))));
    addCommands(cubeIntake.intakeCommand().withTimeout(.7));

    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(secondPiece),
                Waypoint.fromHolonomicPose(secondPieceIntermediate),
                Waypoint.fromHolonomicPose(secondPieceScore))));
    addCommands(cubeIntake.ejectCommand().withTimeout(.4));

    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(secondPieceScore),
                Waypoint.fromDifferentialPose(thirdPieceIntermediate),
                Waypoint.fromHolonomicPose(thirdPiece))));

    addCommands(cubeIntake.intakeCommand().withTimeout(.7));

    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(thirdPiece),
                Waypoint.fromDifferentialPose(thirdPieceReturnIntermediateIntermediate),
                Waypoint.fromHolonomicPose(thirdPieceReturnIntermediate),
                Waypoint.fromHolonomicPose(thirdPieceScore))));
    addCommands(cubeIntake.ejectCommand().withTimeout(.4));
  }
}
