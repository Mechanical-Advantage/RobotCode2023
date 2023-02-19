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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;

public class IntakeAlongFloor extends SequentialCommandGroup {
  public static final double height = 0.15;
  public static final double frontMinX = 0.75;
  public static final double frontMaxX = 1.3;
  public static final double backMinX = -0.45;
  public static final double backMaxX = -1.3;

  /**
   * Moves the arm along the floor based on the position of a joystick while running the gripper.
   */
  public IntakeAlongFloor(
      boolean isFront,
      Arm arm,
      Gripper gripper,
      Objective objective,
      Supplier<Double> axisSupplier) {

    var limiter = new SlewRateLimiter(2.0);
    Supplier<Double> xSupplier =
        () ->
            MathUtil.interpolate(
                isFront ? frontMinX : backMinX, isFront ? frontMaxX : backMaxX, axisSupplier.get());
    var armCommand =
        new FunctionalCommand(
            () -> {
              arm.runPath(
                  new ArmPose(
                      new Translation2d(xSupplier.get(), height),
                      Rotation2d.fromDegrees(isFront ? 0.0 : 180.0)));
            },
            () -> {
              if (arm.isTrajectoryFinished()) {
                arm.runDirect(
                    new ArmPose(
                        new Translation2d(limiter.calculate(xSupplier.get()), height),
                        Rotation2d.fromDegrees(isFront ? 0.0 : 180.0)));
              } else {
                limiter.reset(arm.getSetpoint().endEffectorPosition().getX());
              }
            },
            (interrupted) -> arm.runPath(ArmPose.Preset.HOMED),
            () -> false,
            arm);

    addCommands(
        Commands.waitSeconds(0.1)
            .andThen(
                armCommand.alongWith(
                    gripper.intakeCommand(),
                    Commands.run(() -> objective.lastIntakeFront = true))));
  }
}
