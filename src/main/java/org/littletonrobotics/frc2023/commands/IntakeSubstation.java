// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;

public class IntakeSubstation extends SequentialCommandGroup {
  /** Holds the arm at the position for the a substation and runs the gripper. */
  public IntakeSubstation(
      boolean single, Arm arm, Drive drive, Gripper gripper, ObjectiveTracker objectiveTracker) {
    var armCommand =
        new HoldFlippableArmPreset(
            arm,
            drive,
            single
                ? ArmPose.Preset.SINGLE_SUBTATION.getPose()
                : ArmPose.Preset.DOUBLE_SUBTATION.getPose(),
            Rotation2d.fromDegrees(single ? 90.0 : 0.0));
    addCommands(
        armCommand.alongWith(
            gripper.intakeCommand(),
            Commands.run(() -> objectiveTracker.lastIntakeFront = !armCommand.isFlipped())));
  }
}
