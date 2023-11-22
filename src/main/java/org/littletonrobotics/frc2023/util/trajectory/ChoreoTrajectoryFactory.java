// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory;

import static org.littletonrobotics.frc2023.util.trajectory.CustomSwerveDriveController.DriveDynamicState;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

public class ChoreoTrajectoryFactory {
  private static final ObjectMapper mapper = new ObjectMapper();
  private static final HashMap<String, File> FILE_HASH_MAP = new HashMap<>();
  // load trajectory files into map
  static {
    File trajectory_dir = Filesystem.getDeployDirectory();
    if (trajectory_dir.isDirectory() && trajectory_dir.listFiles() != null) {
      for (File file : trajectory_dir.listFiles()) {
        String fileName = file.getName();
        // check if json
        int pointIndex = fileName.lastIndexOf(".");
        String type = fileName.substring(pointIndex);
        if (!type.equals(".json")) continue;
        // use name of file without ".json"
        String trimmedName = fileName.substring(0, pointIndex - 1);
        FILE_HASH_MAP.put(trimmedName, file);
      }
    }
  }

  // TODO: does this add another unnecessary purpose to this file?

  /**
   * @param trajectoryName Name of trajectory file without ".json"
   * @return Optional of trajectory file
   */
  public static Optional<File> getTrajectoryFile(String trajectoryName) {
    if (!FILE_HASH_MAP.containsKey(trajectoryName)) {
      DriverStation.reportWarning("No trajectory with name: " + trajectoryName, false);
      return Optional.empty();
    }
    return Optional.of(FILE_HASH_MAP.get(trajectoryName));
  }

  /**
   * @param trajectoryFile File to make trajectory from
   * @return
   */
  public static TrajectoryImplementation createTrajectoryFromFile(File trajectoryFile) {
    try {
      return createTrajectoryImplementation(
          mapper.readValue(trajectoryFile, new TypeReference<>() {}));
    } catch (IOException e) {
      e.printStackTrace();
      return createTrajectoryImplementation(Collections.emptyList());
    }
  }

  private static TrajectoryImplementation createTrajectoryImplementation(
      List<ChoreoTrajectoryState> states) {
    return new TrajectoryImplementation() {
      @Override
      public double getDuration() {
        return states.get(states.size() - 1).timestamp();
      }

      @Override
      public Pose2d[] getTrajectoryPoses() {
        return states.stream()
            .map(state -> new Pose2d(state.x(), state.y(), new Rotation2d(state.heading())))
            .toArray(Pose2d[]::new);
      }

      @Override
      public DriveDynamicState sample(double time) {
        ChoreoTrajectoryState before = null;
        ChoreoTrajectoryState after = null;

        for (ChoreoTrajectoryState state : states) {
          if (time == state.timestamp()) {
            return ChoreoTrajectoryState.toDynamicState(state);
          }

          if (state.timestamp() < time) {
            before = state;
          } else {
            after = state;
            break;
          }
        }

        if (before == null) return ChoreoTrajectoryState.toDynamicState(states.get(0));

        if (after == null)
          return ChoreoTrajectoryState.toDynamicState(states.get(states.size() - 1));

        double s = 1 - ((after.timestamp() - time) / (after.timestamp() - before.timestamp()));

        DriveDynamicState beforeState = ChoreoTrajectoryState.toDynamicState(before);
        DriveDynamicState afterState = ChoreoTrajectoryState.toDynamicState(after);

        Pose2d interpolatedPose = beforeState.pose().interpolate(afterState.pose(), s);
        double interpolatedVelocityX =
            MathUtil.interpolate(beforeState.velocityX(), afterState.velocityX(), s);
        double interpolatedVelocityY =
            MathUtil.interpolate(beforeState.velocityY(), afterState.velocityY(), s);
        double interpolatedAngularVelocity =
            MathUtil.interpolate(beforeState.angularVelocity(), afterState.angularVelocity(), s);

        return new DriveDynamicState(
            interpolatedPose,
            interpolatedVelocityX,
            interpolatedVelocityY,
            interpolatedAngularVelocity);
      }
    };
  }

  private record ChoreoTrajectoryState(
      double timestamp,
      double x,
      double y,
      double heading,
      double velocityX,
      double velocityY,
      double angularVelocity) {
    public static DriveDynamicState toDynamicState(ChoreoTrajectoryState state) {
      Pose2d pose = new Pose2d(state.x(), state.y(), new Rotation2d(state.heading()));
      return new DriveDynamicState(
          pose, state.velocityX(), state.velocityY(), state.angularVelocity());
    }
  }
}
