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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.util.GeomUtil;

public class MoveArmWithJoysticks extends CommandBase {
  private static final double maxLinearSpeed = 1.0; // m/s
  private static final double maxWristSpeed = Units.degreesToRadians(180.0);

  private final Arm arm;
  private final Supplier<Double> linearXSupplier;
  private final Supplier<Double> linearYSupplier;
  private final Supplier<Double> wristSupplier;

  /** Translates the arm pose using the joysticks. */
  public MoveArmWithJoysticks(
      Arm arm,
      Supplier<Double> linearXSupplier,
      Supplier<Double> linearYSupplier,
      Supplier<Double> wristSupplier) {
    this.arm = arm;
    this.linearXSupplier = linearXSupplier;
    this.linearYSupplier = linearYSupplier;
    this.wristSupplier = wristSupplier;
  }

  @Override
  public void execute() {
    // Get values from double suppliers
    double linearX = linearXSupplier.get();
    double linearY = linearYSupplier.get();
    double wrist = wristSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(linearX, linearY);
    Rotation2d linearDirection = new Rotation2d(linearX, linearY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, DriveWithJoysticks.deadband);
    wrist = MathUtil.applyDeadband(wrist, DriveWithJoysticks.deadband);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    wrist = Math.copySign(wrist * wrist, wrist);

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0))
            .getTranslation();

    // Convert to normal units
    linearVelocity = linearVelocity.times(maxLinearSpeed);
    var wristVelocity = wrist * maxWristSpeed;

    // Flip wrist velocity for opposite side
    var setpoint = arm.getSetpoint();
    if (setpoint.endEffectorPosition().getX() < 0.0) {
      wristVelocity *= -1;
    }

    // Send command
    arm.runDirect(
        new ArmPose(
            setpoint.endEffectorPosition().plus(linearVelocity.times(Constants.loopPeriodSecs)),
            setpoint
                .globalWristAngle()
                .plus(new Rotation2d(wristVelocity * Constants.loopPeriodSecs))));
  }
}
