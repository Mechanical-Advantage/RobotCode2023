// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Optional;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;

public class ReachForDemoTag extends SequentialCommandGroup {
  private static final double armX = -0.3;
  private static final double armYMin = 0.6;
  private static final double armYMax = 2.0;
  private static final Rotation2d wristRotation = Rotation2d.fromDegrees(180.0);
  private static final TrapezoidProfile.Constraints armProfileConstraints =
      new TrapezoidProfile.Constraints(Units.inchesToMeters(40.0), Units.inchesToMeters(50.0));

  private final AprilTagVision aprilTagVision;
  private TrapezoidProfile.State armHeightState;

  public ReachForDemoTag(Drive drive, Arm arm, AprilTagVision aprilTagVision) {
    this.aprilTagVision = aprilTagVision;
    addCommands(
        Commands.sequence(
                Commands.runOnce(
                    () -> {
                      aprilTagVision.setVisionUpdatesEnabled(false);
                      armHeightState = new TrapezoidProfile.State(armYMin, 0.0);
                    }),
                arm.runPathCommand(ArmPose.Preset.HOMED),
                Commands.parallel(
                    // Turn to tag
                    new DriveToPose(
                        drive,
                        true,
                        () -> {
                          Optional<Pose3d> tagPose = aprilTagVision.getDemoTagPose();
                          if (tagPose.isEmpty()) {
                            return drive.getPose();
                          }
                          Rotation2d rotationToTag =
                              tagPose
                                  .get()
                                  .toPose2d()
                                  .getTranslation()
                                  .minus(drive.getPose().getTranslation())
                                  .getAngle();
                          return new Pose2d(
                              drive.getPose().getTranslation(),
                              rotationToTag.plus(Rotation2d.fromDegrees(180.0)));
                        }),

                    // Move arm to correct height
                    Commands.repeatingSequence(
                        Commands.waitUntil(() -> aprilTagVision.getDemoTagPose().isPresent()),
                        arm.runPathCommand(this::getArmPose),
                        Commands.run(() -> arm.runDirect(getArmPose()), arm)
                            .until(() -> aprilTagVision.getDemoTagPose().isEmpty()),
                        arm.runPathCommand(ArmPose.Preset.HOMED))))
            .finallyDo((boolean interrupted) -> aprilTagVision.setVisionUpdatesEnabled(true)));
  }

  private ArmPose getArmPose() {
    Optional<Pose3d> tagPose = aprilTagVision.getDemoTagPose();
    if (tagPose.isPresent()) {
      double height = tagPose.get().getZ();
      height = MathUtil.clamp(height, armYMin, armYMax);
      armHeightState =
          new TrapezoidProfile(
                  armProfileConstraints, new TrapezoidProfile.State(height, 0.0), armHeightState)
              .calculate(Constants.loopPeriodSecs);
    }
    return new ArmPose(new Translation2d(armX, armHeightState.position), wristRotation);
  }
}
