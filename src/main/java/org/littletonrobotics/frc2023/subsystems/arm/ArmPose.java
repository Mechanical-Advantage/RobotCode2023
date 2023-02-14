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
import org.littletonrobotics.frc2023.commands.DriveToNode;
import org.littletonrobotics.frc2023.commands.MoveArmAlongFloor;

/** Represents a target position for the arm. */
public record ArmPose(Translation2d endEffectorPosition, Rotation2d globalWristAngle) {
  public static enum Preset {
    HOMED(null),
    SCORE_HYBRID(new ArmPose(new Translation2d(0.55, 0.9), Rotation2d.fromDegrees(-90.0))),
    SCORE_MID_CONE(
        new ArmPose(
            new Translation2d(
                DriveToNode.scorePositionX - FieldConstants.Grids.midX - 0.2,
                FieldConstants.Grids.midConeZ + 0.2),
            Rotation2d.fromDegrees(30.0))),
    SCORE_MID_CUBE(
        new ArmPose(
            new Translation2d(
                DriveToNode.scorePositionX - FieldConstants.Grids.midX - 0.3,
                FieldConstants.Grids.midCubeZ + 0.5),
            Rotation2d.fromDegrees(-45.0))),
    SCORE_HIGH_CONE(
        new ArmPose(
            new Translation2d(
                DriveToNode.scorePositionX - FieldConstants.Grids.highX - 0.2,
                FieldConstants.Grids.highConeZ + 0.15),
            Rotation2d.fromDegrees(30.0))),
    SCORE_HIGH_CUBE(
        new ArmPose(
            new Translation2d(
                DriveToNode.scorePositionX - FieldConstants.Grids.highX - 0.3,
                FieldConstants.Grids.highCubeZ + 0.5),
            Rotation2d.fromDegrees(-45.0))),
    SINGLE_SUBTATION(
        new ArmPose(
            new Translation2d(0.49, FieldConstants.LoadingZone.singleSubstationCenterZ),
            new Rotation2d())),
    DOUBLE_SUBTATION(
        new ArmPose(
            new Translation2d(0.49, FieldConstants.LoadingZone.doubleSubstationShelfZ + 0.1),
            new Rotation2d())),
    CUBE_HANDOFF(new ArmPose(new Translation2d(0.3, 0.57), Rotation2d.fromDegrees(-75.0))),
    CONE_HANDOFF(new ArmPose(new Translation2d(-0.31, 0.53), Rotation2d.fromDegrees(175.0))),
    CONE_HANDOFF_RELEASED(
        new ArmPose(new Translation2d(-0.45, 0.6), Rotation2d.fromDegrees(175.0))),
    FLOOR_CLOSE(new ArmPose(new Translation2d(0.8, MoveArmAlongFloor.height), new Rotation2d())),
    FLOOR_CENTER(new ArmPose(new Translation2d(1.0, MoveArmAlongFloor.height), new Rotation2d())),
    FLOOR_FAR(new ArmPose(new Translation2d(1.2, MoveArmAlongFloor.height), new Rotation2d())),
    EJECT(new ArmPose(new Translation2d(0.17, 0.5), Rotation2d.fromDegrees(10.0)));

    private ArmPose pose;

    private Preset(ArmPose pose) {
      this.pose = pose;
    }

    public ArmPose getPose() {
      return pose;
    }

    public static void updateHomedPreset(ArmConfig config) {
      HOMED.pose =
          new ArmPose(
              new Translation2d(
                  config.origin().getX(),
                  config.origin().getY() + config.shoulder().length() - config.elbow().length()),
              new Rotation2d(-Math.PI / 2));
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
