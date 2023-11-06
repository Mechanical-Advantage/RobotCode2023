// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands.autos;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.io.File;
import org.littletonrobotics.frc2023.commands.DriveTrajectory;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.FullStateSwerveTrajectory;

public class TestPathAuto extends SequentialCommandGroup {
  private static final FullStateSwerveTrajectory trajectory =
      FullStateSwerveTrajectory.fromFile(new File(Filesystem.getDeployDirectory(), "NewPath.json"));

  /** Creates a new ScoreAndBalance. */
  public TestPathAuto(Drive drive) {
    addCommands(
        Commands.runOnce(
            () -> drive.setPose(AllianceFlipUtil.apply(trajectory.getStates().get(0).getPose()))));
    addCommands(new DriveTrajectory(drive, trajectory));
  }
}
