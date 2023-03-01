// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntake;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;

public class IntakeCubeHandoff extends SequentialCommandGroup {
  /** Runs the cube intake with the gripper to hand off immediately. */
  public IntakeCubeHandoff(CubeIntake intake, Arm arm, Gripper gripper, Objective objective) {
    addCommands(
        intake
            .runCommand(arm::isTrajectoryFinished)
            .deadlineWith(
                arm.runPathCommand(ArmPose.Preset.CUBE_HANDOFF),
                gripper.intakeCommand(),
                Commands.run(() -> objective.lastIntakeFront = true))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }
}
