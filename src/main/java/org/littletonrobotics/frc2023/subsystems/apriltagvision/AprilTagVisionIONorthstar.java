// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.apriltagvision;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import org.littletonrobotics.frc2023.FieldConstants;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
  private static final int cameraId = 1;
  private static final int cameraResolutionWidth = 1600;
  private static final int cameraResolutionHeight = 1200;
  private static final int cameraResolutionAutoExposure = 1;
  private static final int cameraResolutionExposure = 15;
  private static final double fiducialSize = FieldConstants.aprilTagWidth;

  private final StringSubscriber observationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  public AprilTagVisionIONorthstar(String identifier) {
    var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);

    var configTable = northstarTable.getSubTable("config");
    configTable.getIntegerTopic("camera_id").publish().set(cameraId);
    configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
    configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraResolutionAutoExposure);
    configTable.getIntegerTopic("camera_exposure").publish().set(cameraResolutionExposure);
    configTable.getDoubleTopic("fiducial_size_m").publish().set(fiducialSize);

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getStringTopic("observations")
            .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);
  }

  public void updateInputs(AprilTagVisionIOInputs inputs) {
    var queue = observationSubscriber.readQueue();
    inputs.timestamps = new double[queue.length];
    inputs.frames = new String[queue.length];
    for (int i = 0; i < queue.length; i++) {
      inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
      inputs.frames[i] = queue[i].value;
    }
    inputs.fps = fpsSubscriber.get();
  }
}
