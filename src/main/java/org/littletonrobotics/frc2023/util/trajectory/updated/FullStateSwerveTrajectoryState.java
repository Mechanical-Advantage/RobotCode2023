// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory.updated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

/**
 * Container class for a state in a trajectory which describes the full state of the robot. The
 * structure of this class matches the JSON output format of Choreo.
 *
 * @param timestamp The time at which this state is for
 * @param x The x coordinate of the state (m)
 * @param y The y coordinate of the state (m)
 * @param heading The heading of the state (rad)
 * @param velocityX The linear X velocity of the state (m/s)
 * @param velocityY The linear Y velocity of the state (m/s)
 * @param angularVelocity The angular velocity of the state (rad/s)
 */
public record FullStateSwerveTrajectoryState(
    double timestamp,
    double x,
    double y,
    double heading,
    double velocityX,
    double velocityY,
    double angularVelocity) {
  public Pose2d getPose() {
    return new Pose2d(new Translation2d(x, y), Rotation2d.fromRadians(heading));
  }

  public double getLinearSpeed() {
    return Math.hypot(velocityX, velocityY);
  }

  public Trajectory.State toWpilibState() {
    double linearSpeed = getLinearSpeed();
    return new Trajectory.State(
        timestamp, linearSpeed, 0, getPose(), angularVelocity / linearSpeed);
  }

  public FullStateSwerveTrajectoryState maybeFlip() {
    return new FullStateSwerveTrajectoryState(
        timestamp,
        AllianceFlipUtil.apply(x),
        y,
        AllianceFlipUtil.apply(Rotation2d.fromRadians(heading)).getRadians(),
        AllianceFlipUtil.shouldFlip() ? -velocityX : velocityX,
        velocityY,
        AllianceFlipUtil.shouldFlip() ? -angularVelocity : angularVelocity);
  }
}
