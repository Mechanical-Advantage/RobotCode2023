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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.frc2023.Constants;

public class ArmTrajectoryCache {
  private static final String cacheFilename = "arm_trajectory_cache.json";
  private static final String cacheRequestFilename = "arm_trajectory_cache_request.json";

  private ArmTrajectoryCache() {}

  public static List<ArmTrajectory> loadTrajectories() {
    // Load JSON
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    TrajectoryCacheStore cache;
    try {
      cache =
          mapper.readValue(
              new File(Filesystem.getDeployDirectory(), cacheFilename), TrajectoryCacheStore.class);
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
                      trajectoryCache.finalJointPositions()[1]),
                  Set.of(trajectoryCache.constraintOverrides)));
      List<Vector<N2>> points = new ArrayList<>();
      for (int i = 0; i < trajectoryCache.points().length / 2; i++) {
        points.add(
            VecBuilder.fill(trajectoryCache.points()[i * 2], trajectoryCache.points()[i * 2 + 1]));
      }
      trajectory.setPoints(trajectoryCache.totalTime(), points);
      if (!trajectory.isGenerated() && RobotBase.isSimulation()) {
        // Throw during test
        System.err.println("Invalid trajectory found in arm trajectory cache JSON");
        throw new RuntimeException();
      }
      trajectories.add(trajectory);
    }

    return trajectories;
  }

  /** Pregenerates all trajectories between presets using Kairos. */
  public static void main(String... args) throws IOException, InterruptedException {
    Constants.disableHAL();

    // Load config
    File configFile = Path.of("src", "main", "deploy", Arm.configFilename).toFile();
    String configJson = Files.readString(configFile.toPath());
    ArmConfig config = ArmConfig.loadJson(configFile);
    ArmKinematics kinematics = new ArmKinematics(config);
    ArmPose.Preset.updateHomedPreset(config);

    // Create set of trajectories between presets
    List<TrajectoryCache> allTrajectories = new ArrayList<>();
    List<ArmPose> allPoses = new ArrayList<>();
    for (var preset : ArmPose.Preset.values()) {
      allPoses.add(preset.getPose());
      if (preset.shouldPregenerateFlip()) {
        allPoses.add(preset.getPose().withFlip(true));
      }
    }
    for (var pose0 : allPoses) {
      for (var pose1 : allPoses) {
        if (!pose0.endEffectorPosition().equals(pose1.endEffectorPosition())) {
          Optional<Vector<N2>> preset0Angles = kinematics.inverse(pose0.endEffectorPosition());
          Optional<Vector<N2>> preset1Angles = kinematics.inverse(pose1.endEffectorPosition());
          if (preset0Angles.isPresent() && preset1Angles.isPresent()) {
            // Add to list of trajectories
            allTrajectories.add(
                new TrajectoryCache(
                    new double[] {preset0Angles.get().get(0, 0), preset0Angles.get().get(1, 0)},
                    new double[] {preset1Angles.get().get(0, 0), preset1Angles.get().get(1, 0)},
                    stringSetToArray(
                        Arm.getTrajectoryConstraintOverrides(
                            kinematics, preset0Angles.get(), preset1Angles.get())),
                    0,
                    new double[] {}));
          }
        }
      }
    }

    // Add extra scoring trajectories
    Vector<N2> homedAngles =
        kinematics.inverse(ArmPose.Preset.HOMED.getPose().endEffectorPosition()).get();
    List.of(
            ArmPose.Preset.SCORE_HYBRID.getPose(),
            ArmPose.Preset.SCORE_HYBRID.getPose().withFlip(true),
            ArmPose.Preset.SCORE_MID_CONE.getPose(),
            ArmPose.Preset.SCORE_MID_CONE.getPose().withFlip(true),
            ArmPose.Preset.SCORE_MID_CUBE.getPose(),
            ArmPose.Preset.SCORE_MID_CUBE.getPose().withFlip(true),
            ArmPose.Preset.SCORE_HIGH_CONE.getPose(),
            ArmPose.Preset.SCORE_HIGH_CONE.getPose().withFlip(true),
            ArmPose.Preset.SCORE_HIGH_CUBE.getPose(),
            ArmPose.Preset.SCORE_HIGH_CUBE.getPose().withFlip(true))
        .forEach(
            (ArmPose targetPose) -> {
              double xPosition = targetPose.endEffectorPosition().getX();
              while (true) {
                xPosition += targetPose.endEffectorPosition().getX() > 0.0 ? 0.05 : -0.05;

                boolean lastPoint = false;
                double maxReach =
                    kinematics.calcMaxReachAtHeight(targetPose.endEffectorPosition().getY());
                if (Math.abs(xPosition) > maxReach) {
                  xPosition = targetPose.endEffectorPosition().getX() > 0.0 ? maxReach : -maxReach;
                  lastPoint = true;
                }
                Optional<Vector<N2>> targetAngles =
                    kinematics.inverse(
                        new Translation2d(xPosition, targetPose.endEffectorPosition().getY()));
                if (targetAngles.isEmpty()) {
                  break;
                }

                // Add trajectories between homed and target
                String[] constraintOverrides =
                    stringSetToArray(
                        Arm.getTrajectoryConstraintOverrides(
                            kinematics, targetAngles.get(), homedAngles));
                allTrajectories.add(
                    new TrajectoryCache(
                        new double[] {homedAngles.get(0, 0), homedAngles.get(1, 0)},
                        new double[] {targetAngles.get().get(0, 0), targetAngles.get().get(1, 0)},
                        constraintOverrides,
                        0,
                        new double[] {}));
                allTrajectories.add(
                    new TrajectoryCache(
                        new double[] {targetAngles.get().get(0, 0), targetAngles.get().get(1, 0)},
                        new double[] {homedAngles.get(0, 0), homedAngles.get(1, 0)},
                        constraintOverrides,
                        0,
                        new double[] {}));

                // Exit if last point
                if (lastPoint) {
                  break;
                }
              }
            });

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

    // Save request JSON
    File cacheFile = Path.of(System.getProperty("java.io.tmpdir"), cacheRequestFilename).toFile();
    mapper.writeValue(cacheFile, new TrajectoryCacheStore(configAndPresetHash, allTrajectories));

    // Launch Python (try venv and then default)
    Process venvPython = runPython(true);
    Process systemPython = null;
    if (venvPython.exitValue() != 0) {
      systemPython = runPython(false);
    }

    // Print output
    if (systemPython == null) {
      printOutput(venvPython);
    } else if (systemPython.exitValue() == 0) {
      printOutput(systemPython);
    } else {
      printOutput(venvPython);
      System.out.println();
      printOutput(systemPython);
    }
    int exitValue = systemPython == null ? venvPython.exitValue() : systemPython.exitValue();
    System.out.println(
        "Kairos exited with code "
            + Integer.toString(exitValue)
            + " (Python="
            + (systemPython == null ? "venv" : "system")
            + ")");
    System.exit(exitValue);
  }

  private static Process runPython(boolean useVenv) throws IOException, InterruptedException {
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
        new ProcessBuilder(pythonPath, "kairos" + File.separator + "run_generate_cache.py");
    Process python = pythonBuilder.start();
    python.waitFor();
    return python;
  }

  private static void printOutput(Process process) throws IOException {
    BufferedReader stdoutReader = process.inputReader();
    BufferedReader stderrReader = process.errorReader();
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
  }

  private static String[] stringSetToArray(Set<String> set) {
    String[] array = set.toArray(new String[set.size()]);
    Arrays.sort(array);
    return array;
  }

  private static record TrajectoryCacheStore(
      String configAndPresetHash, List<TrajectoryCache> trajectories) {}

  private static record TrajectoryCache(
      double[] initialJointPositions,
      double[] finalJointPositions,
      String[] constraintOverrides,
      double totalTime,
      double[] points) {}
}
