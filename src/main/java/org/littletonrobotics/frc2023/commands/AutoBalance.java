// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AutoBalance extends CommandBase {
  private static final LoggedTunableNumber speedInchesPerSec =
      new LoggedTunableNumber("AutoBalance/SpeedInchesPerSec", 15.0);
  private static final LoggedTunableNumber positionThresholdDegrees =
      new LoggedTunableNumber("AutoBalance/PositionThresholdDegrees", 3.0);
  private static final LoggedTunableNumber velocityThresholdDegreesPerSec =
      new LoggedTunableNumber("AutoBalance/VelocityThresholdDegreesPerSec", 8.0);

  private final Drive drive;
  private double angleDegrees;

  public AutoBalance(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleDegrees = Double.POSITIVE_INFINITY;
  }

  @Override
  public void execute() {
    // Calculate charge station angle and velocity
    angleDegrees =
        drive.getRotation().getCos() * drive.getPitch().getDegrees()
            + drive.getRotation().getSin() * drive.getRoll().getDegrees();
    double angleVelocityDegreesPerSec =
        drive.getRotation().getCos() * Units.radiansToDegrees(drive.getPitchVelocity())
            + drive.getRotation().getSin() * Units.radiansToDegrees(drive.getRollVelocity());
    boolean shouldStop =
        (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec.get())
            || (angleDegrees > 0.0
                && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec.get());

    // Send velocity to drive
    if (shouldStop) {
      drive.stop();
    } else {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              Units.inchesToMeters(speedInchesPerSec.get()) * (angleDegrees > 0.0 ? -1.0 : 1.0),
              0.0,
              0.0,
              drive.getRotation()));
    }

    // Log data
    Logger.getInstance().recordOutput("AutoBalance/AngleDegrees", angleDegrees);
    Logger.getInstance()
        .recordOutput("AutoBalance/AngleVelocityDegreesPerSec", angleVelocityDegreesPerSec);
    Logger.getInstance().recordOutput("AutoBalance/Stopped", shouldStop);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(angleDegrees) < positionThresholdDegrees.get();
  }
}
