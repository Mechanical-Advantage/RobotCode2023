// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriveWithJoysticks extends CommandBase {
  public static final double matchEndThreshold =
      0.25; // FMS reports "0" ~250ms before the end of the match anyway
  public static final LoggedTunableNumber deadband =
      new LoggedTunableNumber("DriveWithJoysticks/Deadband", 0.1);

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightYSupplier;

  private static final LoggedDashboardChooser<Double> linearSpeedLimitChooser =
      new LoggedDashboardChooser<>("Linear Speed Limit");
  private static final LoggedDashboardChooser<Double> angularSpeedLimitChooser =
      new LoggedDashboardChooser<>("Angular Speed Limit");

  static {
    linearSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
    linearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    linearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    linearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
    angularSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
    angularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    angularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    angularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
  }

  /** Returns whether at least one demo speed limit is active. */
  public static boolean isDemo() {
    return linearSpeedLimitChooser.get() != 1.0 || angularSpeedLimitChooser.get() != 1.0;
  }

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightYSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Go to X right before the end of the match
    if (DriverStation.getMatchTime() >= 0.0 && DriverStation.getMatchTime() < matchEndThreshold) {
      drive.stopWithX();
      return;
    }

    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightY = rightYSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband.get());
    rightY = MathUtil.applyDeadband(rightY, deadband.get());

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightY = Math.copySign(rightY * rightY, rightY);

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0))
            .getTranslation();

    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            rightY * drive.getMaxAngularSpeedRadPerSec());

    // Convert from field relative
    var driveRotation = drive.getRotation();
    if (DriverStation.getAlliance() == Alliance.Red) {
      driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
    }
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            driveRotation);

    // Send to drive
    var driveTranslation = AllianceFlipUtil.apply(drive.getPose().getTranslation());
    if (Math.abs(speeds.vxMetersPerSecond) < 1e-3
        && Math.abs(speeds.vyMetersPerSecond) < 1e-3
        && Math.abs(speeds.omegaRadiansPerSecond) < 1e-3
        && driveTranslation.getX() > FieldConstants.Community.chargingStationInnerX
        && driveTranslation.getX() < FieldConstants.Community.chargingStationOuterX
        && driveTranslation.getY() > FieldConstants.Community.chargingStationRightY
        && driveTranslation.getY() < FieldConstants.Community.chargingStationLeftY) {
      drive.stopWithX();
    } else {
      drive.runVelocity(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
