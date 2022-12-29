// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
@SuppressWarnings("MemberName")
public class CustomHolonomicDriveController {
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private boolean m_enabled = true;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final PIDController m_thetaController;

  /**
   * Constructs a holonomic drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A PID controller to respond to error in angle.
   */
  @SuppressWarnings("ParameterName")
  public CustomHolonomicDriveController(
      PIDController xController, PIDController yController, PIDController thetaController) {
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param poseRef The desired pose.
   * @param linearVelocityRefMeters The linear velocity reference.
   * @param angleRef The angular reference.
   * @param angleVelocityRefRadians The angular velocity reference.
   * @return The next output of the holonomic drive controller.
   */
  @SuppressWarnings("LocalVariableName")
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d poseRef,
      double linearVelocityRefMeters,
      Rotation2d angleRef,
      double angleVelocityRefRadians) {

    // Calculate feedforward velocities (field-relative).
    double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
    double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
    double thetaFF = angleVelocityRefRadians;

    m_poseError = poseRef.relativeTo(currentPose);
    m_rotationError = angleRef.minus(currentPose.getRotation());

    if (!m_enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    }

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
    double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());
    double thetaFeedback =
        m_thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.getRotation());
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param driveState The desired drive trajectory state.
   * @param holonomicRotationState The desired holonomic rotation state.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Trajectory.State driveState,
      RotationSequence.State holonomicRotationState) {
    return calculate(
        currentPose,
        driveState.poseMeters,
        driveState.velocityMetersPerSecond,
        holonomicRotationState.position,
        holonomicRotationState.velocityRadiansPerSec);
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }
}
