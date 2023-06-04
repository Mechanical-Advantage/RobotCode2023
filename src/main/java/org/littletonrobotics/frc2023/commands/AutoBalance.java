// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class AutoBalance extends DriveToPose {
  public AutoBalance(Drive drive) {
    super(
        drive,
        () ->
            new Pose2d(
                new Translation2d(
                    AllianceFlipUtil.apply(
                        MathUtil.interpolate(
                            FieldConstants.Community.chargingStationInnerX,
                            FieldConstants.Community.chargingStationOuterX,
                            0.6)),
                    drive.getPose().getY()),
                drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drive.stopWithX();
  }

  @Override
  public boolean isFinished() {
    return DriverStation.getMatchTime() >= 0.0
        && DriverStation.getMatchTime() < DriveWithJoysticks.matchEndThreshold;
  }
}
