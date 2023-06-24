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
public class WallsideThreePiece extends SequentialCommandGroup {

  private static final Pose2d startPosition =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .5, FieldConstants.Grids.nodeY[1]),
          new Rotation2d(Math.PI));

  private static final Pose2d exitIntermediatePosition =
      new Pose2d(
          new Translation2d(
              FieldConstants.Community.chargingStationInnerX,
              FieldConstants.Community.chargingStationRightY / 2),
          new Rotation2d());

  private static final Pose2d secondPiece =
      new Pose2d(
          new Translation2d(
              FieldConstants.StagingLocations.positionX - .45,
              FieldConstants.StagingLocations.firstY + .35),
          Rotation2d.fromDegrees(-40));

  private static final Pose2d secondPieceIntermediate =
      new Pose2d(
          new Translation2d(
              (FieldConstants.Community.chargingStationInnerX
                      + FieldConstants.Community.chargingStationOuterX)
                  / 2,
              FieldConstants.Community.chargingStationRightY / 2),
          new Rotation2d(Math.PI));

  private static final Pose2d secondPieceScore =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .7, FieldConstants.Grids.nodeY[0] + .15),
          Rotation2d.fromDegrees(-150));

  private static final Pose2d thirdPieceIntermediate =
      new Pose2d(
          new Translation2d(
              (FieldConstants.Community.chargingStationInnerX
                      + FieldConstants.Community.chargingStationOuterX)
                  / 2,
              FieldConstants.Community.chargingStationRightY / 2),
          new Rotation2d());
  private static final Pose2d thirdPieceReturnIntermediateIntermediate =
      new Pose2d(
          new Translation2d(
              FieldConstants.StagingLocations.positionX - 2,
              (FieldConstants.StagingLocations.firstY) - .15),
          new Rotation2d(Math.PI));

  private static final Pose2d thirdPieceReturnIntermediate =
      new Pose2d(
          new Translation2d(
              FieldConstants.Community.chargingStationInnerX - .15,
              (FieldConstants.StagingLocations.firstY) - .15),
          new Rotation2d(Math.PI / 3));

  private static final Pose2d thirdPiece =
      new Pose2d(
          new Translation2d(
              FieldConstants.StagingLocations.positionX - .50,
              (FieldConstants.StagingLocations.firstY + FieldConstants.StagingLocations.separationY)
                  - .35),
          Rotation2d.fromDegrees(40));

  private static final Pose2d thirdPieceScore =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .6, FieldConstants.Grids.nodeY[2]),
          new Rotation2d(Math.PI));

  /** Creates a new WallsideThreePiece. */
  public WallsideThreePiece(Drive drive, CubeIntake cubeIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(startPosition))));

    addCommands(cubeIntake.ejectCommand().withTimeout(.2));

    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(startPosition),
                Waypoint.fromDifferentialPose(exitIntermediatePosition),
                Waypoint.fromHolonomicPose(secondPiece))));
    addCommands(cubeIntake.intakeCommand().withTimeout(.6));

    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(secondPiece),
                Waypoint.fromHolonomicPose(secondPieceIntermediate),
                Waypoint.fromHolonomicPose(secondPieceScore))));
    addCommands(cubeIntake.ejectCommand().withTimeout(.2));

    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(secondPieceScore),
                Waypoint.fromDifferentialPose(thirdPieceIntermediate),
                Waypoint.fromHolonomicPose(thirdPiece))));

    addCommands(cubeIntake.intakeCommand().withTimeout(.6));

    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(thirdPiece),
                Waypoint.fromDifferentialPose(thirdPieceReturnIntermediateIntermediate),
                Waypoint.fromHolonomicPose(thirdPieceReturnIntermediate),
                Waypoint.fromHolonomicPose(thirdPieceScore))));
    addCommands(cubeIntake.ejectCommand().withTimeout(.2));
  }
}
