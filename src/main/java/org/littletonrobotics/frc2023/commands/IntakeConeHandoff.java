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
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.coneintake.ConeIntake;
import org.littletonrobotics.frc2023.subsystems.coneintake.ConeIntake.Mode;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;

public class IntakeConeHandoff extends SequentialCommandGroup {
  /** Runs the cone intake and hands off to the gripper when finished. */
  public IntakeConeHandoff(
      ConeIntake intake,
      Arm arm,
      Gripper gripper,
      ObjectiveTracker objectiveTracker,
      Supplier<Boolean> trigger) {
    addCommands(
        // Initial deploy, wait to complete
        intake.setModeCommand(Mode.INTAKING),
        Commands.runOnce(() -> arm.runPath(ArmPose.Preset.CONE_HANDOFF), arm),
        Commands.waitUntil(() -> arm.isTrajectoryFinished() || !trigger.get()),
        Commands.either(
                // Continue if button still active
                Commands.sequence(
                    Commands.waitUntil(() -> !trigger.get()),
                    Commands.runOnce(() -> objectiveTracker.lastIntakeFront = false),
                    intake.setModeCommand(Mode.HANDOFF_START),
                    Commands.sequence(
                            Commands.waitSeconds(0.3),
                            intake.setModeCommand(Mode.HANDOFF_EJECT),
                            arm.runPathCommand(ArmPose.Preset.CONE_HANDOFF_RELEASED))
                        .deadlineWith(gripper.intakeCommand())),

                // Exit if button released before deploy complete
                Commands.none(),

                // Get button value
                () -> trigger.get())

            // Return to homed
            .finallyDo(
                (interrupted) -> {
                  intake.setMode(Mode.NEUTRAL);
                  arm.runPath(ArmPose.Preset.HOMED);
                }));
  }
}
