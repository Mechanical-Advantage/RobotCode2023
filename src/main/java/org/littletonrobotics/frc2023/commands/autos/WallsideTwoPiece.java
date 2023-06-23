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
import org.littletonrobotics.frc2023.commands.AutoBalance;
import org.littletonrobotics.frc2023.commands.DriveTrajectory;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntake;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;

public class WallsideTwoPiece extends SequentialCommandGroup {
  private static final Pose2d startPosition =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .5, FieldConstants.Grids.nodeY[1]),
          new Rotation2d(Math.PI));

  private static final Pose2d intermediatePosition =
      new Pose2d(
          new Translation2d(
              FieldConstants.Community.chargingStationInnerX,
              FieldConstants.Community.chargingStationRightY / 2),
          new Rotation2d());

  private static final Pose2d returnIntermediatePosition =
      new Pose2d(
          new Translation2d(
              (FieldConstants.Community.chargingStationInnerX
                      + FieldConstants.Community.chargingStationOuterX)
                  / 2,
              FieldConstants.Community.chargingStationRightY / 2),
          new Rotation2d(Math.PI));

  private static final Pose2d returnPosition =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .7, FieldConstants.Grids.nodeY[0] + .15),
          Rotation2d.fromDegrees(-150));

  private static final Pose2d endPosition =
      new Pose2d(
          new Translation2d(
              FieldConstants.StagingLocations.positionX - .45,
              FieldConstants.StagingLocations.firstY + .35),
          Rotation2d.fromDegrees(-40));

  private static final Pose2d chargeStationIntermediate =
      new Pose2d(
          new Translation2d(
              FieldConstants.Grids.outerX + 1,
              FieldConstants.Community.chargingStationLeftY
                  - ((FieldConstants.Community.chargingStationLength / 4) * 3)
                  - .25),
          new Rotation2d());

  private static final Pose2d chargeStation =
      new Pose2d(
          new Translation2d(
              (FieldConstants.Community.chargingStationInnerX
                      + FieldConstants.Community.chargingStationOuterX)
                  / 2,
              FieldConstants.Community.chargingStationLeftY
                  - ((FieldConstants.Community.chargingStationLength / 4) * 3)
                  - .25),
          new Rotation2d(Math.PI));

  /** Creates a new WallsideScoreMidMobility. */
  public WallsideTwoPiece(Drive drive, CubeIntake cubeIntake, boolean shouldBalance) {
    addCommands(Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(startPosition))));
    addCommands(cubeIntake.ejectCommand().withTimeout(.5));
    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(startPosition),
                Waypoint.fromDifferentialPose(intermediatePosition),
                Waypoint.fromHolonomicPose(endPosition))));
    addCommands(cubeIntake.intakeCommand().withTimeout(1));
    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(endPosition),
                Waypoint.fromDifferentialPose(returnIntermediatePosition),
                Waypoint.fromHolonomicPose(returnPosition))));
    addCommands(cubeIntake.ejectCommand().withTimeout(.5));

    if (shouldBalance) {
      addCommands(
          new DriveTrajectory(
              drive,
              List.of(
                  Waypoint.fromHolonomicPose(returnPosition),
                  Waypoint.fromDifferentialPose(chargeStationIntermediate, new Rotation2d(Math.PI)),
                  Waypoint.fromHolonomicPose(chargeStation))));
      addCommands(new AutoBalance(drive));
    }
  }
}
