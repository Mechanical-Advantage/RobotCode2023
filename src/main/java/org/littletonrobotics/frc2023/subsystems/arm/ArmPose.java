// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.commands.AutoScore;

/** Represents a target position for the arm. */
public record ArmPose(Translation2d endEffectorPosition, Rotation2d globalWristAngle) {
  public static enum Preset {
    HOMED(null),
    SCORE_HYBRID(
        new ArmPose(
            new Translation2d(
                    Math.max(
                            AutoScore.minDriveX,
                            FieldConstants.Grids.lowX
                                - AutoScore.hybridRelativePosition.getX()
                                + AutoScore.minArmExtension)
                        - FieldConstants.Grids.lowX,
                    0.0)
                .plus(AutoScore.hybridRelativePosition),
            AutoScore.hybridWristAngle)),
    SCORE_MID_CUBE(
        new ArmPose(
            new Translation2d(
                    Math.max(
                            AutoScore.minDriveX,
                            FieldConstants.Grids.midX
                                - AutoScore.cubeRelativePosition.getX()
                                + AutoScore.minArmExtension)
                        - FieldConstants.Grids.midX,
                    FieldConstants.Grids.midCubeZ)
                .plus(AutoScore.cubeRelativePosition),
            AutoScore.cubeWristAngle)),
    SCORE_HIGH_CUBE(
        new ArmPose(
            new Translation2d(
                    Math.max(
                            AutoScore.minDriveX,
                            FieldConstants.Grids.highX
                                - AutoScore.cubeRelativePosition.getX()
                                + AutoScore.minArmExtension)
                        - FieldConstants.Grids.highX,
                    FieldConstants.Grids.highCubeZ)
                .plus(AutoScore.cubeRelativePosition),
            AutoScore.cubeWristAngle)),
    SCORE_MID_UPRIGHT_CONE(
        new ArmPose(
            new Translation2d(
                    Math.max(
                            AutoScore.minDriveX,
                            FieldConstants.Grids.midX
                                - AutoScore.uprightConeRelativePosition.getX()
                                + AutoScore.minArmExtension)
                        - FieldConstants.Grids.midX,
                    FieldConstants.Grids.midConeZ)
                .plus(AutoScore.uprightConeRelativePosition),
            AutoScore.uprightConeWristAngle)),
    SCORE_HIGH_UPRIGHT_CONE(
        new ArmPose(
            new Translation2d(
                    Math.max(
                            AutoScore.minDriveX,
                            FieldConstants.Grids.highX
                                - AutoScore.uprightConeRelativePosition.getX()
                                + AutoScore.minArmExtension)
                        - FieldConstants.Grids.highX,
                    FieldConstants.Grids.highConeZ)
                .plus(AutoScore.uprightConeRelativePosition),
            AutoScore.uprightConeWristAngle)),
    SCORE_MID_TIPPED_CONE(
        new ArmPose(
            new Translation2d(
                    Math.max(
                            AutoScore.minDriveX,
                            FieldConstants.Grids.midX
                                - AutoScore.tippedConeRelativePosition.getX()
                                + AutoScore.minArmExtension)
                        - FieldConstants.Grids.midX,
                    FieldConstants.Grids.midConeZ)
                .plus(AutoScore.tippedConeRelativePosition),
            AutoScore.tippedConeWristAngle)),
    SCORE_HIGH_TIPPED_CONE(
        new ArmPose(
            new Translation2d(
                    Math.max(
                            AutoScore.minDriveX,
                            FieldConstants.Grids.highX
                                - AutoScore.tippedConeRelativePosition.getX()
                                + AutoScore.minArmExtension)
                        - FieldConstants.Grids.highX,
                    FieldConstants.Grids.highConeZ)
                .plus(AutoScore.tippedConeRelativePosition),
            AutoScore.tippedConeWristAngle)),
    SINGLE_SUBTATION(
        new ArmPose(
            new Translation2d(0.49, FieldConstants.LoadingZone.singleSubstationCenterZ - 0.15),
            Rotation2d.fromDegrees(20.0))),
    DOUBLE_SUBTATION(
        new ArmPose(
            new Translation2d(0.49, FieldConstants.LoadingZone.doubleSubstationShelfZ + 0.1),
            new Rotation2d())),
    CUBE_HANDOFF(new ArmPose(new Translation2d(0.42, 0.65), Rotation2d.fromDegrees(-85.0)), false),
    FLOOR_CONE(new ArmPose(new Translation2d(-0.5, 0.18), Rotation2d.fromDegrees(-165.0)), false),
    EJECT(new ArmPose(new Translation2d(0.25, 0.5), Rotation2d.fromDegrees(10.0)));

    private ArmPose pose;
    private boolean pregenerateFlip;

    private Preset(ArmPose pose) {
      this(pose, true);
    }

    private Preset(ArmPose pose, boolean pregenerateFlip) {
      this.pose = pose;
      this.pregenerateFlip = pregenerateFlip;
    }

    public ArmPose getPose() {
      return pose;
    }

    public boolean shouldPregenerateFlip() {
      return pregenerateFlip;
    }

    public static void updateHomedPreset(ArmConfig config) {
      HOMED.pose =
          new ArmPose(
              new Translation2d(
                  config.origin().getX(),
                  config.origin().getY() + config.shoulder().length() - config.elbow().length()),
              Rotation2d.fromDegrees(-180.0));
    }
  }

  static double wristLength = 0.0;

  public Translation2d wristPosition() {
    return new Pose2d(endEffectorPosition, globalWristAngle)
        .transformBy(new Transform2d(new Translation2d(wristLength, 0.0), new Rotation2d()))
        .getTranslation();
  }

  public ArmPose withFlip(boolean flip) {
    return flip
        ? new ArmPose(
            new Translation2d(-this.endEffectorPosition.getX(), this.endEffectorPosition.getY()),
            new Rotation2d(-this.globalWristAngle.getCos(), this.globalWristAngle.getSin()))
        : this;
  }
}
