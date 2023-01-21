// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;

public class HoldPose extends CommandBase {
  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("HoldPose/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("HoldPose/DriveKd");
  private static final LoggedTunableNumber turnKp = new LoggedTunableNumber("HoldPose/TurnKp");
  private static final LoggedTunableNumber turnKd = new LoggedTunableNumber("HoldPose/TurnKd");
  private static final LoggedTunableNumber distanceTolerance =
      new LoggedTunableNumber("HoldPose/DistanceTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("HoldPose/ThetaTolerance");

  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

  private final Drive drive;
  private final Pose2d target;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023P:
        driveKp.initDefault(2.5);
        driveKd.initDefault(0.0);
        turnKp.initDefault(7.0);
        turnKd.initDefault(0.0);
        distanceTolerance.initDefault(0.02);
        thetaTolerance.initDefault(Units.degreesToRadians(3.0));
        break;
      default:
        break;
    }
  }

  /** Creates a HoldPose command. */
  public HoldPose(Drive drive, Pose2d target) {
    this.drive = drive;
    this.target = target;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    xController.reset();
    yController.reset();
    thetaController.reset();

    // Reset PID gains
    xController.setP(driveKp.get());
    xController.setD(driveKd.get());
    yController.setP(driveKp.get());
    yController.setD(driveKd.get());
    thetaController.setP(turnKp.get());
    thetaController.setD(turnKd.get());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    // Update from tunable numbers
    if (driveKd.hasChanged(hashCode()) || driveKp.hasChanged(hashCode())) {
      xController.setP(driveKp.get());
      xController.setD(driveKd.get());
      yController.setP(driveKp.get());
      yController.setD(driveKd.get());
      thetaController.setP(turnKp.get());
      thetaController.setD(turnKd.get());
    }

    // Calculate velocity
    if (drive.getPose().getTranslation().getDistance(target.getTranslation())
            < distanceTolerance.get()
        && drive.getRotation().minus(target.getRotation()).getRadians() < thetaTolerance.get()) {
      drive.stop();
    } else {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xController.calculate(drive.getPose().getX(), target.getX()),
              yController.calculate(drive.getPose().getY(), target.getY()),
              thetaController.calculate(
                  drive.getPose().getRotation().getRadians(), target.getRotation().getRadians()),
              drive.getRotation()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
