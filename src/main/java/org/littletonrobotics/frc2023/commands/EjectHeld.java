// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper.EjectSpeed;

public class EjectHeld extends SequentialCommandGroup {
  /** Ejects the current game piece out the front or back. */
  public EjectHeld(boolean isFront, Arm arm, Gripper gripper) {
    addCommands(
        arm.runPathCommand(ArmPose.Preset.EJECT.getPose().withFlip(!isFront))
            .andThen(gripper.ejectCommand(EjectSpeed.SLOW))
            .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));
  }
}
