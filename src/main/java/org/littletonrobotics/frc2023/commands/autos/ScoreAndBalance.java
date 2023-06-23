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

public class ScoreAndBalance extends SequentialCommandGroup {
  private static final Pose2d startPosition =
      new Pose2d(
          new Translation2d(FieldConstants.Grids.outerX + .5, FieldConstants.Grids.nodeY[4]),
          new Rotation2d(Math.PI));

  private static final Pose2d endPosition =
      new Pose2d(
          (FieldConstants.Community.chargingStationInnerX
                  + FieldConstants.Community.chargingStationOuterX)
              / 2,
          (FieldConstants.Community.chargingStationRightY
                  + FieldConstants.Community.chargingStationLeftY)
              / 2,
          new Rotation2d(Math.PI));

  /** Creates a new ScoreAndBalance. */
  public ScoreAndBalance(Drive drive, CubeIntake cubeIntake) {
    addCommands(Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(startPosition))));
    addCommands(cubeIntake.ejectCommand().withTimeout(2));
    addCommands(
        new DriveTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(startPosition),
                Waypoint.fromHolonomicPose(endPosition))));
    addCommands(new AutoBalance(drive));
  }
}