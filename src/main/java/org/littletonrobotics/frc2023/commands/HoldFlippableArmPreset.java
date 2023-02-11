// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.Arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class HoldFlippableArmPreset extends CommandBase {
  private final Arm arm;
  private final Drive drive;
  private final ArmPose pose;
  private final Rotation2d fieldRotation;

  private boolean lastIsFlipped;

  /**
   * Moves the arm to the specified pose while flipping to align with the specified direction on the
   * field.
   */
  public HoldFlippableArmPreset(Arm arm, Drive drive, ArmPose pose, Rotation2d fieldRotation) {
    this.arm = arm;
    this.drive = drive;
    this.pose = pose;
    this.fieldRotation = fieldRotation;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    lastIsFlipped = isFlipped();
    arm.runPath(pose.withFlip(lastIsFlipped));
  }

  @Override
  public void execute() {
    if (lastIsFlipped != isFlipped()) {
      lastIsFlipped = !lastIsFlipped;
      arm.runPath(pose.withFlip(lastIsFlipped));
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.runPath(ArmPose.Preset.HOMED);
  }

  public boolean isFlipped() {
    return drive.getRotation().minus(AllianceFlipUtil.apply(fieldRotation)).getCos() < 0.0;
  }
}
