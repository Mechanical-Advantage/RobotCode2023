// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.objectivetracker;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Paths;

public class NodeSelectorIOServer implements NodeSelectorIO {
  private final IntegerPublisher publisher;
  private final IntegerSubscriber subscriber;

  public NodeSelectorIOServer() {
    // Create publisher and subscriber
    var table = NetworkTableInstance.getDefault().getTable("nodeselector");
    publisher = table.getIntegerTopic("robot_to_dashboard").publish();
    subscriber = table.getIntegerTopic("dashboard_to_robot").subscribe(-1);

    // Start server
    var app =
        Javalin.create(
            config -> {
              config.staticFiles.add(
                  Paths.get(
                          Filesystem.getDeployDirectory().getAbsolutePath().toString(),
                          "nodeselector")
                      .toString(),
                  Location.EXTERNAL);
            });
    app.start(5800);
  }

  public void updateInputs(NodeSelectorIOInputs inputs) {
    for (var value : subscriber.readQueueValues()) {
      inputs.selected = value;
    }
  }

  public void setSelected(long selected) {
    publisher.set(selected);
  }
}
