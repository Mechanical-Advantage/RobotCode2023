// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory.updated;

import static org.littletonrobotics.frc2023.util.AllianceFlipUtil.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class CustomSwerveDriveController {
  private PIDController xController, yController, thetaController;

  private Pose2d m_poseError;
  private Rotation2d m_rotationError;

  public CustomSwerveDriveController(
      PIDController xController, PIDController yController, PIDController thetaController) {
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;

    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ChassisSpeeds calculate(Pose2d currentPose, DriveDynamicState state) {
    // Calculate feedforward velocities (field-relative).
    double xFF = state.velocityX();
    double yFF = state.velocityY();
    double thetaFF = state.angularVelocity();

    Pose2d poseRef = state.pose();

    m_poseError = poseRef.relativeTo(currentPose);
    m_rotationError = poseRef.getRotation().minus(currentPose.getRotation());

    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = yController.calculate(currentPose.getY(), poseRef.getY());
    double thetaFeedback =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), poseRef.getRotation().getRadians());

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.getRotation());
  }

  public record DriveDynamicState(
      Pose2d pose, double velocityX, double velocityY, double angularVelocity) {
    public DriveDynamicState maybeFlip() {
      double x = apply(pose.getX());
      double y = pose.getY();
      Rotation2d heading = apply(pose.getRotation());
      return new DriveDynamicState(
          new Pose2d(x, y, heading),
          shouldFlip() ? -velocityX : velocityX,
          velocityY,
          shouldFlip() ? -angularVelocity : angularVelocity);
    }
  }
}
