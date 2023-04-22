// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.Constants.Mode;
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
  private static final LoggedTunableNumber stoppedFinalMinTime =
      new LoggedTunableNumber("AutoBalance/StoppedFinalMinTime", 0.75);

  private final Drive drive;
  private double angleDegrees;
  private double lastStoppedFinalTimestamp;

  public AutoBalance(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleDegrees = Double.POSITIVE_INFINITY;
    lastStoppedFinalTimestamp = -1000.0;
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
    boolean shouldStopTemporary =
        (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec.get())
            || (angleDegrees > 0.0
                && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec.get());
    boolean shouldStopFinal = Math.abs(angleDegrees) < positionThresholdDegrees.get();
    if (shouldStopFinal) {
      lastStoppedFinalTimestamp = Timer.getFPGATimestamp();
    }

    // Send velocity to drive
    if (shouldStopTemporary) {
      drive.stop();
    } else if (Timer.getFPGATimestamp() - lastStoppedFinalTimestamp < stoppedFinalMinTime.get()) {
      drive.stopWithX();
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
    Logger.getInstance().recordOutput("AutoBalance/StoppedTemporary", shouldStopTemporary);
    Logger.getInstance().recordOutput("AutoBalance/StoppedFinal", shouldStopFinal);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
  }

  @Override
  public boolean isFinished() {
    return Constants.getMode() == Mode.SIM // Can't balance in sim
        || (DriverStation.getMatchTime() >= 0.0
            && DriverStation.getMatchTime() < DriveWithJoysticks.matchEndThreshold);
  }
}
