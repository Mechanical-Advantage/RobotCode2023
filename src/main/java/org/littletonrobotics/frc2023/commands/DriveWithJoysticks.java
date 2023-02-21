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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class DriveWithJoysticks extends CommandBase {
  public static final double deadband = 0.1;
  public static final double minExtensionMaxLinearAcceleration = Units.inchesToMeters(900.0);
  public static final double fullExtensionMaxLinearAcceleration = Units.inchesToMeters(200.0);
  public static final double fullExtensionMaxAngularVelocity = Units.degreesToRadians(90.0);
  public static final double sniperModeLinearPercent = 0.5;
  public static final double sniperModeAngularPercent = 0.5;

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightYSupplier;
  private final Supplier<Boolean> sniperModeSupplier;
  private final Supplier<Boolean> robotRelativeOverride;
  private final Supplier<Double> armExtensionPercentSupplier;
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

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

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightYSupplier,
      Supplier<Boolean> sniperModeSupplier,
      Supplier<Boolean> robotRelativeOverride,
      Supplier<Double> armExtensionPercentSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
    this.sniperModeSupplier = sniperModeSupplier;
    this.robotRelativeOverride = robotRelativeOverride;
    this.armExtensionPercentSupplier = armExtensionPercentSupplier;
  }

  @Override
  public void initialize() {
    lastSpeeds = new ChassisSpeeds();
  }

  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightY = rightYSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
    rightY = MathUtil.applyDeadband(rightY, deadband);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightY = Math.copySign(rightY * rightY, rightY);

    // Apply speed limits
    linearMagnitude *= linearSpeedLimitChooser.get();
    rightY *= angularSpeedLimitChooser.get();
    if (sniperModeSupplier.get()) {
      linearMagnitude *= sniperModeLinearPercent;
      rightY *= sniperModeAngularPercent;
    }

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
    if (!robotRelativeOverride.get()) {
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
    }

    // Apply acceleration and velocity limits based on arm extension
    double maxLinearAcceleration =
        MathUtil.interpolate(
            minExtensionMaxLinearAcceleration,
            fullExtensionMaxLinearAcceleration,
            armExtensionPercentSupplier.get());
    double maxAngularVelocity =
        MathUtil.interpolate(
            drive.getMaxAngularSpeedRadPerSec(),
            fullExtensionMaxAngularVelocity,
            armExtensionPercentSupplier.get());
    speeds =
        new ChassisSpeeds(
            MathUtil.clamp(
                speeds.vxMetersPerSecond,
                lastSpeeds.vxMetersPerSecond - maxLinearAcceleration * Constants.loopPeriodSecs,
                lastSpeeds.vxMetersPerSecond + maxLinearAcceleration * Constants.loopPeriodSecs),
            MathUtil.clamp(
                speeds.vyMetersPerSecond,
                lastSpeeds.vyMetersPerSecond - maxLinearAcceleration * Constants.loopPeriodSecs,
                lastSpeeds.vyMetersPerSecond + maxLinearAcceleration * Constants.loopPeriodSecs),
            MathUtil.clamp(speeds.omegaRadiansPerSecond, -maxAngularVelocity, maxAngularVelocity));
    lastSpeeds = speeds;

    // Send to drive
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
