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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private static final double ambiguityThreshold = 0.4;
  private static final Pose3d[] cameraPoses;

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputsAutoLogged[] inputs;

  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};

  static {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        cameraPoses = new Pose3d[] {new Pose3d(0.0, 0.0, 1.0, new Rotation3d())};
        break;
      default:
        cameraPoses = new Pose3d[] {};
        break;
    }
  }

  public AprilTagVision(AprilTagVisionIO... io) {
    this.io = io;
    inputs = new AprilTagVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputsAutoLogged();
    }
  }

  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs("AprilTagVision/" + Integer.toString(i), inputs[i]);
    }

    // Loop over instances
    Pose2d currentPose = poseSupplier.get();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    List<Pose2d> visionPoses = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      // Loop over frames
      for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
        var timestamp = inputs[instanceIndex].timestamps[frameIndex];
        var observationString = inputs[instanceIndex].frames[frameIndex];
        if (observationString.length() == 0) {
          continue;
        }

        // Parse string to double array
        String[] stringComponents = observationString.split(",");
        double[] values = new double[stringComponents.length];
        for (int i = 0; i < stringComponents.length; i++) {
          values[i] = Double.parseDouble(stringComponents[i]);
        }

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
          if (error0 < error1 * ambiguityThreshold) {
            robotPose = robotPose0;
          } else if (error1 < error0 * ambiguityThreshold) {
            robotPose = robotPose1;
          } else if (robotPose0.getTranslation().getDistance(currentPose.getTranslation())
              < robotPose1.getTranslation().getDistance(currentPose.getTranslation())) {
            robotPose = robotPose0;
          } else {
            robotPose = robotPose1;
          }

          // Add to vision updates
          visionUpdates.add(
              new TimestampedVisionUpdate(timestamp, robotPose, VecBuilder.fill(0.1, 0.1, 0.1)));
          visionPoses.add(robotPose);
        }
      }
    }

    // Log poses
    if (visionPoses.size() > 0) {
      Logger.getInstance()
          .recordOutput(
              "Odometry/VisionPoses", visionPoses.toArray(new Pose2d[visionPoses.size()]));
    }

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
