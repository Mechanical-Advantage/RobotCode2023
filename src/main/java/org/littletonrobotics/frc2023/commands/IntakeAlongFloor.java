// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;

public class IntakeAlongFloor extends SequentialCommandGroup {
  /**
   * Moves the arm along the floor based on the position of a joystick while running the gripper.
   */
  public IntakeAlongFloor(
      boolean isFront,
      Arm arm,
      Gripper gripper,
      ObjectiveTracker objectiveTracker,
      Supplier<Double> axisSupplier) {
    addCommands(
        Commands.waitSeconds(0.1)
            .andThen(
                new MoveArmAlongFloor(arm, axisSupplier, true)
                    .alongWith(
                        gripper.intakeCommand(),
                        Commands.run(() -> objectiveTracker.lastIntakeFront = true))));
  }
}
