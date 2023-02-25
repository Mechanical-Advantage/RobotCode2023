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
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO.AprilTagVisionIOInputs;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.PolynomialRegression;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private static final double ambiguityThreshold = 0.15;
  private static final double targetLogTimeSecs = 0.05;
  private static final Pose3d[] cameraPoses;
  private static final PolynomialRegression xyStdDevModel;
  private static final PolynomialRegression thetaStdDevModel;

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        cameraPoses =
            new Pose3d[] {
              new Pose3d(
                  Units.inchesToMeters(4.0),
                  Units.inchesToMeters(-12.5),
                  Units.inchesToMeters(19.75),
                  new Rotation3d(0.0, Units.degreesToRadians(15.0), 0.0)),
              new Pose3d(
                  Units.inchesToMeters(-4.0),
                  Units.inchesToMeters(-11.5),
                  Units.inchesToMeters(19.0),
                  new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0))),
              new Pose3d(
                  Units.inchesToMeters(-11.5),
                  Units.inchesToMeters(7.5),
                  Units.inchesToMeters(6.5),
                  new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0))
                      .rotateBy(new Rotation3d(0.0, Units.degreesToRadians(-14.0), 0.0)))
            };
        xyStdDevModel =
            new PolynomialRegression(
                new double[] {
                  0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                  3.223358, 4.093358, 4.726358
                },
                new double[] {
                  0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.0695, 0.046, 0.1245, 0.0815, 0.193
                },
                1);
        thetaStdDevModel =
            new PolynomialRegression(
                new double[] {
                  0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                  3.223358, 4.093358, 4.726358
                },
                new double[] {
                  0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068
                },
                1);
        break;
      default:
        cameraPoses = new Pose3d[] {};
        xyStdDevModel =
            new PolynomialRegression(new double[] {0.0, 1.0}, new double[] {0.1, 0.1}, 1);
        thetaStdDevModel =
            new PolynomialRegression(new double[] {0.0, 1.0}, new double[] {0.1, 0.1}, 1);
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

  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
    }

    // Loop over instances
    Pose2d currentPose = poseSupplier.get();
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
            } else if (Math.abs(
                    robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
                < Math.abs(
                    robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())) {
              cameraPose = cameraPose0;
              robotPose = robotPose0;
            } else {
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
        double xyStdDev = xyStdDevModel.predict(avgDistance) / tagPoses.size();
        double thetaStdDev = thetaStdDevModel.predict(avgDistance) / tagPoses.size();
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
