// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.List;

public class FullStateSwerveTrajectory {
  private static final ObjectMapper mapper = new ObjectMapper();

  private final List<FullStateSwerveTrajectoryState> states;

  public FullStateSwerveTrajectory(List<FullStateSwerveTrajectoryState> states) {
    this.states = List.copyOf(states);
  }

  /**
   * Get the state that should be used for a given timestamp. If the time is greater than the time
   * of the last state, return the last state.
   *
   * @param timestamp Time (s)
   * @return
   */
  public FullStateSwerveTrajectoryState sample(double timestamp) {
    FullStateSwerveTrajectoryState before = null;
    FullStateSwerveTrajectoryState after = null;

    for (FullStateSwerveTrajectoryState state : this.states) {
      // If requested time is equal to one of the state times, we have found the right state
      if (timestamp == state.timestamp()) {
        return state;
      }

      if (state.timestamp() < timestamp) {
        before = state;
      } else {
        after = state;
        break;
      }
    }

    if (before == null) {
      return states.get(0);
    }

    if (after == null) {
      return states.get(states.size() - 1);
    }

    double s = 1 - ((after.timestamp() - timestamp) / (after.timestamp() - before.timestamp()));

    Pose2d interpolatedPose = lerp(before.getPose(), after.getPose(), s);
    double interpolatedVelocityX = lerp(before.velocityX(), after.velocityX(), s);
    double interpolatedVelocityY = lerp(before.velocityY(), after.velocityY(), s);
    double interpolatedAngularVelocity = lerp(before.angularVelocity(), after.angularVelocity(), s);

    return new FullStateSwerveTrajectoryState(
        timestamp,
        interpolatedPose.getX(),
        interpolatedPose.getY(),
        interpolatedPose.getRotation().getRadians(),
        interpolatedVelocityX,
        interpolatedVelocityY,
        interpolatedAngularVelocity);
  }

  public List<FullStateSwerveTrajectoryState> getStates() {
    return List.copyOf(this.states);
  }

  public List<Trajectory.State> getAsWpilibStates() {
    return this.states.stream().map(FullStateSwerveTrajectoryState::toWpilibState).toList();
  }

  public double getDuration() {
    return this.states.get(this.states.size() - 1).timestamp();
  }

  public static FullStateSwerveTrajectory fromFile(File file) {
    try {
      return new FullStateSwerveTrajectory(mapper.readValue(file, new TypeReference<>() {}));
    } catch (IOException e) {
      e.printStackTrace();
      return new FullStateSwerveTrajectory(Collections.emptyList());
    }
  }

  private static double lerp(double startValue, double endValue, double t) {
    return startValue + (endValue - startValue) * t;
  }

  /**
   * Linearly interpolates between two poses.
   *
   * @param startValue The start pose.
   * @param endValue The end pose.
   * @param t The fraction for interpolation.
   * @return The interpolated pose.
   */
  private static Pose2d lerp(Pose2d startValue, Pose2d endValue, double t) {
    return startValue.plus((endValue.minus(startValue)).times(t));
  }
}
