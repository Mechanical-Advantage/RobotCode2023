// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;

public class MoveArmAlongFloor extends CommandBase {
  public static final double height = 0.15;
  public static final double frontMinX = 0.75;
  public static final double frontMaxX = 1.3;
  public static final double backMinX = -0.65;
  public static final double backMaxX = -1.3;

  private final Arm arm;
  private final Supplier<Double> extensionPercentSupplier;
  private final boolean isFront;
  private final SlewRateLimiter limiter = new SlewRateLimiter(1.0);
  private double startX = 0.0;

  /** Moves the arm along the floor based on the position of a joystick. */
  public MoveArmAlongFloor(Arm arm, Supplier<Double> extensionPercentSupplier, boolean isFront) {
    this.arm = arm;
    this.extensionPercentSupplier = extensionPercentSupplier;
    this.isFront = isFront;
  }

  @Override
  public void initialize() {
    startX = getX();
    arm.runPath(
        new ArmPose(
            new Translation2d(startX, height), Rotation2d.fromDegrees(isFront ? 0.0 : 180.0)));
  }

  @Override
  public void execute() {
    if (arm.isTrajectoryFinished()) {
      arm.runDirect(
          new ArmPose(
              new Translation2d(limiter.calculate(getX()), height),
              Rotation2d.fromDegrees(isFront ? 0.0 : 180.0)));
    } else {
      limiter.reset(startX);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.runPath(ArmPose.Preset.HOMED);
  }

  private double getX() {
    return MathUtil.interpolate(
        isFront ? frontMinX : backMinX,
        isFront ? frontMaxX : backMaxX,
        extensionPercentSupplier.get());
  }
}
