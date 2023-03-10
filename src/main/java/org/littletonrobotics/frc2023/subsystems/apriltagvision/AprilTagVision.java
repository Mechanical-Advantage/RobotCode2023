// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.apriltagvision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO.AprilTagVisionIOInputs;
import org.littletonrobotics.frc2023.subsystems.drive.Module;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private static final double ambiguityThreshold = 0.15;
  private static final double targetLogTimeSecs = 0.1;
  private static final Pose3d[] cameraPoses;
  private static final double xyStdDevCoefficient;
  private static final double thetaStdDevCoefficient;

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        cameraPoses =
            new Pose3d[] {
              // Back left (left facing)
              new Pose3d(
                  Units.inchesToMeters(-9.25),
                  Units.inchesToMeters(10.75),
                  Units.inchesToMeters(6.627) + Module.getWheelRadius(),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(83.312)))),

              // Back right (right facing)
              new Pose3d(
                  Units.inchesToMeters(-9.163),
                  Units.inchesToMeters(-11.495),
                  Units.inchesToMeters(7.127) + Module.getWheelRadius(),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-83.312)))),

              // Front left (forward facing)
              new Pose3d(
                  Units.inchesToMeters(10.5),
                  Units.inchesToMeters(9.0),
                  Units.inchesToMeters(7.627) + Module.getWheelRadius(),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-35.124)))),

              // Back right (back facing)
              new Pose3d(
                  Units.inchesToMeters(-11.192),
                  Units.inchesToMeters(-9.34),
                  Units.inchesToMeters(6.627) + Module.getWheelRadius(),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(144.03))))
            };
        xyStdDevCoefficient = 0.01;
        thetaStdDevCoefficient = 0.01;
        break;
      default:
        cameraPoses = new Pose3d[] {};
        xyStdDevCoefficient = 0.01;
        thetaStdDevCoefficient = 0.01;
        break;
    }
  }

  public AprilTagVision(AprilTagVisionIO... io) {
    this.io = io;
    inputs = new AprilTagVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputs();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }

    // Create map of last detection times for tags
    FieldConstants.aprilTags
        .getTags()
        .forEach(
            (AprilTag tag) -> {
              lastTagDetectionTimes.put(tag.ID, 0.0);
            });
  }

  public void setDataInterface(Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.visionConsumer = visionConsumer;
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {

      // Loop over frames
      for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
        var timestamp = inputs[instanceIndex].timestamps[frameIndex];
        var values = inputs[instanceIndex].frames[frameIndex];

        // Switch based on number of poses
        Pose3d cameraPose = null;
        Pose2d robotPose = null;
        switch ((int) values[0]) {
          case 1:
            // One pose (multi-tag), use directly
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose =
                cameraPose
                    .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                    .toPose2d();
            break;

          case 2:
            // Two poses (one tag), disambiguate
            double error0 = values[1];
            double error1 = values[9];
            Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Pose2d robotPose0 =
                cameraPose0
                    .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                    .toPose2d();
            Pose2d robotPose1 =
                cameraPose1
                    .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                    .toPose2d();

            // Select pose using projection errors
            if (error0 < error1 * ambiguityThreshold) {
              cameraPose = cameraPose0;
              robotPose = robotPose0;
            } else if (error1 < error0 * ambiguityThreshold) {
              cameraPose = cameraPose1;
              robotPose = robotPose1;
            }
            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose == null) {
          continue;
        }

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) values[i]);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Add to vision updates
        double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        visionUpdates.add(
            new TimestampedVisionUpdate(
                timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        allRobotPoses.add(robotPose);

        // Log data from instance
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/LatencySecs",
                Timer.getFPGATimestamp() - timestamp);
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose", robotPose);
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
      }

      // If no frames from instances, clear robot pose
      if (inputs[instanceIndex].timestamps.length == 0) {
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose",
                new double[] {});
      }

      // If no recent frames from instance, clear tag poses
      if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagPoses",
                new double[] {});
      }
    }

    // Log robot poses
    Logger.getInstance()
        .recordOutput(
            "AprilTagVision/RobotPoses", allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        allTagPoses.add(FieldConstants.aprilTags.getTagPose(detectionEntry.getKey()).get());
      }
    }
    Logger.getInstance()
        .recordOutput(
            "AprilTagVision/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

    // Send results to pose esimator
    visionConsumer.accept(visionUpdates);
  }
}
