// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.apriltagvision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO.AprilTagVisionIOInputs;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.PolynomialRegression;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private static final double ambiguityThreshold = 0.15;
  private static final double targetLogTimeSecs = 0.1;
  private static final Pose3d[] cameraPoses;
  private static final PolynomialRegression xyStdDevModel;
  private static final PolynomialRegression thetaStdDevModel;

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private Map<Integer, Double> lastDetectionTimeIds = new HashMap<>();

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023P:
        cameraPoses =
            new Pose3d[] {
              new Pose3d(
                  Units.inchesToMeters(12.5),
                  Units.inchesToMeters(6.0),
                  Units.inchesToMeters(9.5),
                  new Rotation3d(0.0, 0.0, Math.PI)
                      .rotateBy(new Rotation3d(0.0, Units.degreesToRadians(25.0), 0.0)))
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

    // Create map of last detection times
    FieldConstants.aprilTags
        .keySet()
        .forEach(
            (Integer id) -> {
              lastDetectionTimeIds.put(id, 0.0);
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
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      List<Pose2d> visionPose2ds = new ArrayList<>();
      List<Pose3d> tagPose3ds = new ArrayList<>();
      List<Integer> tagIds = new ArrayList<>();

      // Loop over frames
      for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
        var timestamp = inputs[instanceIndex].timestamps[frameIndex];
        var values = inputs[instanceIndex].frames[frameIndex];
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/LatencySecs",
                Timer.getFPGATimestamp() - timestamp);

        // Loop over observations
        for (int i = 0; i < values.length; i += 15) {
          // Get observation data
          int tagId = (int) values[i];
          var pose0 =
              openCVPoseToWPILibPose(
                  VecBuilder.fill(values[i + 1], values[i + 2], values[i + 3]),
                  VecBuilder.fill(values[i + 4], values[i + 5], values[i + 6]));
          var error0 = values[i + 7];
          var pose1 =
              openCVPoseToWPILibPose(
                  VecBuilder.fill(values[i + 8], values[i + 9], values[i + 10]),
                  VecBuilder.fill(values[i + 11], values[i + 12], values[i + 13]));
          var error1 = values[i + 14];

          // Calculate robot poses
          var fieldToTag = FieldConstants.aprilTags.get(tagId);
          if (fieldToTag == null) {
            continue;
          }
          var robotPose0 =
              fieldToTag
                  .transformBy(GeomUtil.pose3dToTransform3d(pose0).inverse())
                  .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                  .toPose2d();
          var robotPose1 =
              fieldToTag
                  .transformBy(GeomUtil.pose3dToTransform3d(pose1).inverse())
                  .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                  .toPose2d();

          // Choose better pose
          Pose2d robotPose;
          Pose3d tagPose;
          if (error0 < error1 * ambiguityThreshold) {
            robotPose = robotPose0;
            tagPose = pose0;
          } else if (error1 < error0 * ambiguityThreshold) {
            robotPose = robotPose1;
            tagPose = pose1;
          } else if (Math.abs(
                  robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
              < Math.abs(robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())) {
            robotPose = robotPose0;
            tagPose = pose0;
          } else {
            robotPose = robotPose1;
            tagPose = pose1;
          }

          // Log tag pose
          tagPose3ds.add(tagPose);
          tagIds.add(tagId);
          lastDetectionTimeIds.put(tagId, Timer.getFPGATimestamp());

          // Add to vision updates
          double tagDistance = tagPose.getTranslation().getNorm();
          double xyStdDev = xyStdDevModel.predict(tagDistance);
          double thetaStdDev = thetaStdDevModel.predict(tagDistance);
          visionUpdates.add(
              new TimestampedVisionUpdate(
                  timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
          visionPose2ds.add(robotPose);
        }
      }

      // Log poses
      boolean hasFrames = inputs[instanceIndex].timestamps.length > 0;
      if (hasFrames) {
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPoses",
                visionPose2ds.toArray(new Pose2d[visionPose2ds.size()]));
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagPoses",
                tagPose3ds.toArray(new Pose3d[tagPose3ds.size()]));
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagIDs",
                tagIds.stream().mapToLong(Long::valueOf).toArray());
      }
    }

    // Log target poses
    List<Pose3d> targetPose3ds = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastDetectionTimeIds.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        targetPose3ds.add(FieldConstants.aprilTags.get(detectionEntry.getKey()));
      }
    }
    Logger.getInstance()
        .recordOutput(
            "AprilTagVision/TargetPoses", targetPose3ds.toArray(new Pose3d[targetPose3ds.size()]));

    // Send results to pose esimator
    visionConsumer.accept(visionUpdates);
  }

  private static Pose3d openCVPoseToWPILibPose(Vector<N3> tvec, Vector<N3> rvec) {
    return new Pose3d(
        new Translation3d(tvec.get(2, 0), -tvec.get(0, 0), -tvec.get(1, 0)),
        new Rotation3d(
            VecBuilder.fill(rvec.get(2, 0), -rvec.get(0, 0), -rvec.get(1, 0)),
            Math.sqrt(
                Math.pow(rvec.get(0, 0), 2)
                    + Math.pow(rvec.get(1, 0), 2)
                    + Math.pow(rvec.get(2, 0), 2))));
  }
}
