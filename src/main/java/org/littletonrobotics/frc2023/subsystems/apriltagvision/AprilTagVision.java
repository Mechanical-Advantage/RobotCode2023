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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import org.littletonrobotics.frc2023.subsystems.drive.Module;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private static final double ambiguityThreshold = 0.15;
  private static final double targetLogTimeSecs = 0.1;
  private static final double fieldBorderMargin = 0.5;
  private static final double zMargin = 0.75;
  private static final double demoTagPosePersistenceSecs = 0.5;
  private static final Pose3d[] cameraPoses;
  private static final double xyStdDevCoefficient;
  private static final double thetaStdDevCoefficient;

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  private boolean enableVisionUpdates = true;
  private Alert enableVisionUpdatesAlert =
      new Alert("Vision updates are temporarily disabled.", AlertType.WARNING);
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  private Pose3d demoTagPose = null;
  private double lastDemoTagPoseTimestamp = 0.0;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        cameraPoses =
            new Pose3d[] {
              // Front left (forward facing, camera 6)
              new Pose3d(
                  Units.inchesToMeters(9.875),
                  Units.inchesToMeters(9.55),
                  Units.inchesToMeters(6.752) + Module.getWheelRadius(),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-35.0)))),

              // Back right (right facing, camera 4)
              new Pose3d(
                  Units.inchesToMeters(-10.375),
                  Units.inchesToMeters(-10.242),
                  Units.inchesToMeters(7.252) + Module.getWheelRadius(),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-82.829)))),

              // Back left (left facing, camera 5)
              new Pose3d(
                  Units.inchesToMeters(-10.15),
                  Units.inchesToMeters(9.7),
                  Units.inchesToMeters(5.752) + Module.getWheelRadius(),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(82.829)))),

              // Back right (back facing, camera 3)
              new Pose3d(
                  Units.inchesToMeters(-10.5),
                  Units.inchesToMeters(-9.25),
                  Units.inchesToMeters(6.252) + Module.getWheelRadius(),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(145.0))))
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
    System.out.println("[Init] Creating AprilTagVision");
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
      Consumer<List<TimestampedVisionUpdate>> visionConsumer, Supplier<Pose2d> poseSupplier) {
    this.visionConsumer = visionConsumer;
    this.poseSupplier = poseSupplier;
  }

  /** Returns the field relative pose of the demo tag. */
  public Optional<Pose3d> getDemoTagPose() {
    return Optional.ofNullable(demoTagPose);
  }

  /** Sets whether vision updates for odometry are enabled. */
  public void setVisionUpdatesEnabled(boolean enabled) {
    enableVisionUpdates = enabled;
    enableVisionUpdatesAlert.set(!enabled);
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {

      // Loop over frames
      for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
        var timestamp = inputs[instanceIndex].timestamps[frameIndex];
        var values = inputs[instanceIndex].frames[frameIndex];

        // Exit if blank frame
        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        // Switch based on number of poses
        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;
        switch ((int) values[0]) {
          case 1:
            // One pose (multi-tag), use directly
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose3d =
                cameraPose.transformBy(
                    GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());
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
            Pose3d robotPose3d0 =
                cameraPose0.transformBy(
                    GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());
            Pose3d robotPose3d1 =
                cameraPose1.transformBy(
                    GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());

            // Select pose using projection errors
            if (error0 < error1 * ambiguityThreshold) {
              cameraPose = cameraPose0;
              robotPose3d = robotPose3d0;
            } else if (error1 < error0 * ambiguityThreshold) {
              cameraPose = cameraPose1;
              robotPose3d = robotPose3d1;
            }
            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if robot pose is off the field
        if (robotPose3d.getX() < -fieldBorderMargin
            || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose3d.getY() < -fieldBorderMargin
            || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
            || robotPose3d.getZ() < -zMargin
            || robotPose3d.getZ() > zMargin) {
          continue;
        }

        // Get 2D robot pose
        Pose2d robotPose = robotPose3d.toPose2d();

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
        allRobotPoses3d.add(robotPose3d);

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
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose3d",
                robotPose3d);
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
      }

      // Record demo tag pose
      if (inputs[instanceIndex].demoFrame.length > 0) {
        var values = inputs[instanceIndex].demoFrame;
        double error0 = values[0];
        double error1 = values[8];
        Pose3d fieldToCameraPose =
            new Pose3d(poseSupplier.get())
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]));
        Pose3d fieldToTagPose0 =
            fieldToCameraPose.transformBy(
                new Transform3d(
                    new Translation3d(values[1], values[2], values[3]),
                    new Rotation3d(new Quaternion(values[4], values[5], values[6], values[7]))));
        Pose3d fieldToTagPose1 =
            fieldToCameraPose.transformBy(
                new Transform3d(
                    new Translation3d(values[9], values[10], values[11]),
                    new Rotation3d(
                        new Quaternion(values[12], values[13], values[14], values[15]))));
        Pose3d fieldToTagPose;

        // Find best pose
        if (demoTagPose == null && error0 < error1) {
          fieldToTagPose = fieldToTagPose0;
        } else if (demoTagPose == null && error0 >= error1) {
          fieldToTagPose = fieldToTagPose1;
        } else if (error0 < error1 * ambiguityThreshold) {
          fieldToTagPose = fieldToTagPose0;
        } else if (error1 < error0 * ambiguityThreshold) {
          fieldToTagPose = fieldToTagPose1;
        } else {
          var pose0Quaternion = fieldToTagPose0.getRotation().getQuaternion();
          var pose1Quaternion = fieldToTagPose1.getRotation().getQuaternion();
          var referenceQuaternion = demoTagPose.getRotation().getQuaternion();
          double pose0Distance =
              Math.acos(
                  pose0Quaternion.getW() * referenceQuaternion.getW()
                      + pose0Quaternion.getX() * referenceQuaternion.getX()
                      + pose0Quaternion.getY() * referenceQuaternion.getY()
                      + pose0Quaternion.getZ() * referenceQuaternion.getZ());
          double pose1Distance =
              Math.acos(
                  pose1Quaternion.getW() * referenceQuaternion.getW()
                      + pose1Quaternion.getX() * referenceQuaternion.getX()
                      + pose1Quaternion.getY() * referenceQuaternion.getY()
                      + pose1Quaternion.getZ() * referenceQuaternion.getZ());
          if (pose0Distance > Math.PI / 2) {
            pose0Distance = Math.PI - pose0Distance;
          }
          if (pose1Distance > Math.PI / 2) {
            pose1Distance = Math.PI - pose1Distance;
          }
          if (pose0Distance < pose1Distance) {
            fieldToTagPose = fieldToTagPose0;
          } else {
            fieldToTagPose = fieldToTagPose1;
          }
        }

        // Save pose
        if (fieldToTagPose != null) {
          demoTagPose = fieldToTagPose;
          lastDemoTagPoseTimestamp = Timer.getFPGATimestamp();
        }
      }

      // If no frames from instances, clear robot pose
      if (inputs[instanceIndex].timestamps.length == 0) {
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose",
                new double[] {});
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose3d",
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

    // Clear demo tag pose
    if (Timer.getFPGATimestamp() - lastDemoTagPoseTimestamp > demoTagPosePersistenceSecs) {
      demoTagPose = null;
    }

    // Log robot poses
    Logger.getInstance()
        .recordOutput(
            "AprilTagVision/RobotPoses", allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));
    Logger.getInstance()
        .recordOutput(
            "AprilTagVision/RobotPoses3d",
            allRobotPoses3d.toArray(new Pose3d[allRobotPoses3d.size()]));

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

    // Log demo tag pose
    if (demoTagPose == null) {
      Logger.getInstance().recordOutput("AprilTagVision/DemoTagPose", new double[] {});
    } else {
      Logger.getInstance().recordOutput("AprilTagVision/DemoTagPose", demoTagPose);
    }
    Logger.getInstance().recordOutput("AprilTagVision/DemoTagPoseId", new long[] {29});

    // Send results to pose esimator
    if (enableVisionUpdates) {
      visionConsumer.accept(visionUpdates);
    }
  }
}
