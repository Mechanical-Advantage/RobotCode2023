// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.hash.Hashing;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.frc2023.Constants;

public class ArmTrajectoryCache {
  private static final String cacheFilename = "arm_trajectory_cache.json";

  private ArmTrajectoryCache() {}

  public static List<ArmTrajectory> loadTrajectories() {
    // Load JSON
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    TrajectoryCacheStore cache;
    try {
      cache =
          mapper.readValue(
              Path.of("src", "main", "deploy", cacheFilename).toFile(), TrajectoryCacheStore.class);
    } catch (IOException e) {
      throw new RuntimeException("Failed to parse arm trajectory cache JSON");
    }

    // Create trajectory objects
    List<ArmTrajectory> trajectories = new ArrayList<>();
    for (var trajectoryCache : cache.trajectories()) {
      var trajectory =
          new ArmTrajectory(
              new ArmTrajectory.Parameters(
                  VecBuilder.fill(
                      trajectoryCache.initialJointPositions()[0],
                      trajectoryCache.initialJointPositions()[1]),
                  VecBuilder.fill(
                      trajectoryCache.finalJointPositions()[0],
                      trajectoryCache.finalJointPositions()[1])));
      List<Vector<N2>> points = new ArrayList<>();
      for (int i = 0; i < trajectoryCache.points().length / 2; i++) {
        points.add(
            VecBuilder.fill(trajectoryCache.points()[i * 2], trajectoryCache.points()[i * 2 + 1]));
      }
      trajectory.setPoints(trajectoryCache.totalTime(), points);
      if (!trajectory.isGenerated()) {
        DriverStation.reportError("Invalid trajectory found in arm trajectory cache JSON", false);
      }
      trajectories.add(trajectory);
    }

    return trajectories;
  }

  public static void main(String... args) throws IOException, InterruptedException {
    Constants.disableHAL();

    // Load config
    File configFile = Path.of("src", "main", "deploy", Arm.configFilename).toFile();
    String configJson = Files.readString(configFile.toPath());
    ArmConfig config = ArmConfig.loadJson(configFile);
    ArmKinematics kinematics = new ArmKinematics(config);
    ArmPose.Preset.updateHomedPreset(config);

    // Create set of trajectories
    List<TrajectoryCache> allTrajectories = new ArrayList<>();
    List<ArmPose> allPoses = new ArrayList<>();
    for (var preset : ArmPose.Preset.values()) {
      allPoses.add(preset.getPose());
      allPoses.add(preset.getPose().withFlip(true));
    }
    for (var pose0 : allPoses) {
      for (var pose1 : allPoses) {
        if (!pose0.equals(pose1)) {
          Optional<Vector<N2>> preset0Angles = kinematics.inverse(pose0.endEffectorPosition());
          Optional<Vector<N2>> preset1Angles = kinematics.inverse(pose1.endEffectorPosition());
          if (preset0Angles.isPresent() && preset1Angles.isPresent()) {
            double[] initialJointPositions =
                new double[] {preset0Angles.get().get(0, 0), preset0Angles.get().get(1, 0)};
            double[] finalJointPositions =
                new double[] {preset1Angles.get().get(0, 0), preset1Angles.get().get(1, 0)};

            // Check if already generated
            boolean identicalFound = false;
            for (var otherTrajectory : allTrajectories) {
              if (Math.abs(initialJointPositions[0] - otherTrajectory.initialJointPositions()[0])
                      < Arm.trajectoryCacheMarginRadians
                  && Math.abs(initialJointPositions[1] - otherTrajectory.initialJointPositions()[1])
                      < Arm.trajectoryCacheMarginRadians
                  && Math.abs(finalJointPositions[0] - otherTrajectory.finalJointPositions()[0])
                      < Arm.trajectoryCacheMarginRadians
                  && Math.abs(finalJointPositions[1] - otherTrajectory.finalJointPositions()[1])
                      < Arm.trajectoryCacheMarginRadians) {
                identicalFound = true;
                break;
              }
            }

            // Add to list of trajectories
            if (!identicalFound) {
              allTrajectories.add(
                  new TrajectoryCache(
                      initialJointPositions, finalJointPositions, 0, new double[] {}));
            }
          }
        }
      }
    }

    // Calculate hash
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    String trajectoryString = mapper.writeValueAsString(allTrajectories);
    String configAndPresetHash =
        Hashing.sha256()
            .hashString(configJson + trajectoryString, StandardCharsets.UTF_8)
            .toString();

    // Compare to existing hash
    TrajectoryCacheStore existingCache = null;
    boolean generateNew = false;
    try {
      existingCache =
          mapper.readValue(
              Path.of("src", "main", "deploy", cacheFilename).toFile(), TrajectoryCacheStore.class);
    } catch (FileNotFoundException e) {
      generateNew = true;
      System.out.println(
          "No existing cache, generating trajectories (count="
              + Integer.toString(allTrajectories.size())
              + ", hash="
              + configAndPresetHash
              + ")");
    }
    if (existingCache != null) {
      generateNew = !existingCache.configAndPresetHash().equals(configAndPresetHash);
      if (generateNew) {
        System.out.println(
            "Cache outdated, generating trajectories (count="
                + Integer.toString(allTrajectories.size())
                + ", hash="
                + configAndPresetHash
                + ")");
      } else {
        System.out.println(
            "Cache up-to-date, skipping trajectory generation (count="
                + Integer.toString(allTrajectories.size())
                + ", hash="
                + configAndPresetHash
                + ")");
      }
    }
    if (!generateNew) System.exit(0);

    // Launch Python (try venv and then default)
    String requestJson =
        mapper.writeValueAsString(new TrajectoryCacheStore(configAndPresetHash, allTrajectories));
    boolean venv = true;
    Process python = runPython(true, requestJson);
    if (python.exitValue() != 0) {
      venv = false;
      python = runPython(false, requestJson);
    }

    // Print output
    BufferedReader stdoutReader = python.inputReader();
    BufferedReader stderrReader = python.errorReader();
    while (true) {
      String line = stdoutReader.readLine();
      if (line == null) {
        break;
      }
      System.out.println("Output: " + line);
    }
    while (true) {
      String line = stderrReader.readLine();
      if (line == null) {
        break;
      }
      System.out.println("Error: " + line);
    }
    System.out.println(
        "Kairos exited with code "
            + Integer.toString(python.exitValue())
            + " (Python="
            + (venv ? "venv" : "system")
            + ")");
    System.exit(python.exitValue());
  }

  private static Process runPython(boolean useVenv, String requestJson)
      throws IOException, InterruptedException {
    ProcessBuilder pythonBuilder;
    String pythonPath = "python";
    if (useVenv) {
      if (System.getProperty("os.name").startsWith("Windows")) {
        pythonPath = "./venv/Scripts/python";
      } else {
        pythonPath = "./venv/bin/python";
      }
    }
    pythonBuilder =
        new ProcessBuilder(
            pythonPath, "kairos" + File.separator + "run_generate_cache.py", requestJson);
    Process python = pythonBuilder.start();
    python.waitFor();
    return python;
  }

  private static record TrajectoryCacheStore(
      String configAndPresetHash, List<TrajectoryCache> trajectories) {}

  private static record TrajectoryCache(
      double[] initialJointPositions,
      double[] finalJointPositions,
      double totalTime,
      double[] points) {}
}
