// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeMap;
import java.util.stream.Collectors;

/** Generator for creating a drive trajectory and rotation sequence from a series of waypoints. */
public class CustomTrajectoryGenerator {
  private Trajectory driveTrajectory = new Trajectory(List.of(new Trajectory.State()));
  private RotationSequence holonomicRotationSequence = new RotationSequence(new TreeMap<>());

  /**
   * Generates a drive trajectory and holonomic rotation sequence from a series of waypoints,
   * combining quintic and cubic splines when necessary. Please note the following limitations:
   *
   * <p>1) The drive rotations for the start and end waypoints are required for trajectory
   * generation. If not specified by the user, these rotations will be created automatically based
   * on the position of the nearest waypoint.
   *
   * <p>2) Transitions between quintic and cubic splines may produce high accelerations due to
   * unrealistic changes in curvature. Please account for this effect when tuning feedback
   * controllers.
   *
   * <p>3) The holonomic rotation sequence attempts to minimize the angular acceleration between
   * points, but does not accept velocity or acceleration constraints as inputs. Ensure that the
   * change in rotation between adjacent waypoints is reasonable.
   *
   * <p>4) The robot's holonomic rotation always assumed to match its drive rotation when applying
   * kinematics constraints. Please use sufficient margins to allow for changes in angular position
   * and velocity.
   *
   * @param config Trajectory configuration
   * @param waypoints A series of waypoints
   */
  public void generate(TrajectoryConfig config, List<Waypoint> waypoints) {
    if (waypoints.size() < 2) {
      throw new InvalidParameterException(
          "Please include at least 2 waypoints to generate a trajectory.");
    }

    // Generate drive waypoints
    List<Translation2d> driveTranslations =
        waypoints.stream().map(waypoint -> waypoint.getTranslation()).collect(Collectors.toList());
    List<Optional<Rotation2d>> driveRotations =
        waypoints.stream()
            .map(waypoint -> waypoint.getDriveRotation())
            .collect(Collectors.toList());
    driveRotations.remove(0);
    driveRotations.remove(driveRotations.size() - 1);

    // Add first drive waypoint
    if (waypoints.get(0).getDriveRotation().isPresent()) {
      driveRotations.add(0, waypoints.get(0).getDriveRotation());
    } else {
      driveRotations.add(
          0,
          Optional.of(
              waypoints
                  .get(1)
                  .getTranslation()
                  .minus(waypoints.get(0).getTranslation())
                  .getAngle()));
    }

    // Add last drive waypoint
    if (waypoints.get(waypoints.size() - 1).getDriveRotation().isPresent()) {
      driveRotations.add(waypoints.get(waypoints.size() - 1).getDriveRotation());
    } else {
      driveRotations.add(
          Optional.of(
              waypoints
                  .get(waypoints.size() - 1)
                  .getTranslation()
                  .minus(waypoints.get(waypoints.size() - 2).getTranslation())
                  .getAngle()));
    }

    // Generate drive trajectory
    driveTrajectory = new Trajectory();
    boolean firstSubTrajectory = true;
    boolean nextQuintic = driveRotations.get(1).isPresent();
    int index = 1;
    int subTrajectoryStart = 0;
    int subTrajectoryEnd = 0;
    while (true) {
      boolean generateSubTrajectory = false;
      boolean lastWaypoint = index == waypoints.size() - 1;
      if (nextQuintic) {
        if (lastWaypoint || driveRotations.get(index).isEmpty()) { // Found a translation or end of
          // waypoints
          generateSubTrajectory = true;
          subTrajectoryEnd = lastWaypoint ? index : index - 1;
        }
      } else {
        if (driveRotations.get(index).isPresent()) { // Found a pose
          generateSubTrajectory = true;
          subTrajectoryEnd = index;
        }
      }

      if (generateSubTrajectory) {
        // Prepare sub-trajectory config
        TrajectoryConfig subConfig = copyConfig(config);
        if (!firstSubTrajectory) {
          subConfig.setStartVelocity(
              driveTrajectory
                  .getStates()
                  .get(driveTrajectory.getStates().size() - 1)
                  .velocityMetersPerSecond);
        }
        if (!lastWaypoint) {
          subConfig.setEndVelocity(subConfig.getMaxVelocity());
        }
        firstSubTrajectory = false;

        // Generate sub-trajectory
        if (nextQuintic) {
          List<Pose2d> quinticWaypoints = new ArrayList<>();
          for (int i = subTrajectoryStart; i < subTrajectoryEnd + 1; i++) {
            quinticWaypoints.add(new Pose2d(driveTranslations.get(i), driveRotations.get(i).get()));
          }
          driveTrajectory =
              driveTrajectory.concatenate(
                  TrajectoryGenerator.generateTrajectory(quinticWaypoints, subConfig));
        } else {
          List<Translation2d> cubicInteriorWaypoints = new ArrayList<>();
          for (int i = subTrajectoryStart + 1; i < subTrajectoryEnd; i++) {
            cubicInteriorWaypoints.add(driveTranslations.get(i));
          }
          driveTrajectory =
              driveTrajectory.concatenate(
                  TrajectoryGenerator.generateTrajectory(
                      new Pose2d(
                          driveTranslations.get(subTrajectoryStart),
                          driveRotations.get(subTrajectoryStart).get()),
                      cubicInteriorWaypoints,
                      new Pose2d(
                          driveTranslations.get(subTrajectoryEnd),
                          driveRotations.get(subTrajectoryEnd).get()),
                      subConfig));
        }

        // Break if complete
        if (lastWaypoint) {
          break;
        }

        // Prepare for next trajectory
        nextQuintic = !nextQuintic;
        subTrajectoryStart = subTrajectoryEnd;
      }

      index++;
    }

    // Find holonmic waypoints
    TreeMap<Double, Rotation2d> holonomicWaypoints = new TreeMap<>();
    int stateIndex = 0;
    for (int waypointIndex = 0; waypointIndex < driveTranslations.size(); waypointIndex++) {
      double timestamp;
      if (waypointIndex == 0) {
        timestamp = 0.0;
      } else if (waypointIndex == driveTranslations.size() - 1) {
        timestamp = driveTrajectory.getTotalTimeSeconds();
      } else {
        while (!driveTrajectory
            .getStates()
            .get(stateIndex)
            .poseMeters
            .getTranslation()
            .equals(driveTranslations.get(waypointIndex))) {
          stateIndex++;
        }
        timestamp = driveTrajectory.getStates().get(stateIndex).timeSeconds;
      }

      if (waypoints.get(waypointIndex).getHolonomicRotation().isPresent()) {
        holonomicWaypoints.put(
            timestamp, waypoints.get(waypointIndex).getHolonomicRotation().get());
      }
    }
    holonomicRotationSequence = new RotationSequence(holonomicWaypoints);
  }

  private TrajectoryConfig copyConfig(TrajectoryConfig config) {
    TrajectoryConfig newConfig =
        new TrajectoryConfig(config.getMaxVelocity(), config.getMaxAcceleration());
    newConfig.addConstraints(config.getConstraints());
    newConfig.setStartVelocity(config.getStartVelocity());
    newConfig.setEndVelocity(config.getEndVelocity());
    newConfig.setReversed(config.isReversed());
    return newConfig;
  }

  /** Returns the generated drive trajectory. */
  public Trajectory getDriveTrajectory() {
    return driveTrajectory;
  }

  /** Returns the generated holonomic rotation sequence. */
  public RotationSequence getHolonomicRotationSequence() {
    return holonomicRotationSequence;
  }
}
