// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.objectivetracker;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Paths;

public class NodeSelectorIOServer implements NodeSelectorIO {
  private final IntegerPublisher nodePublisher;
  private final IntegerSubscriber nodeSubscriber;
  private final BooleanPublisher coneTippedPublisher;
  private final BooleanSubscriber coneTippedSubscriber;
  private final IntegerPublisher timePublisher;
  private final BooleanPublisher isAutoPublisher;

  public NodeSelectorIOServer() {
    System.out.println("[Init] Creating NodeSelectorIOServer");

    // Create publisher and subscriber
    var table = NetworkTableInstance.getDefault().getTable("nodeselector");
    nodePublisher = table.getIntegerTopic("node_robot_to_dashboard").publish();
    nodeSubscriber = table.getIntegerTopic("node_dashboard_to_robot").subscribe(-1);
    coneTippedPublisher = table.getBooleanTopic("cone_tipped_robot_to_dashboard").publish();
    coneTippedSubscriber = table.getBooleanTopic("cone_tipped_dashboard_to_robot").subscribe(false);
    timePublisher = table.getIntegerTopic("match_time").publish();
    isAutoPublisher = table.getBooleanTopic("is_auto").publish();

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
    timePublisher.set((long) Math.ceil(Math.max(0.0, DriverStation.getMatchTime())));
    isAutoPublisher.set(DriverStation.isAutonomous());
    for (var value : nodeSubscriber.readQueueValues()) {
      inputs.selectedNode = value;
    }
    for (var value : coneTippedSubscriber.readQueueValues()) {
      inputs.coneTipped = value ? 1 : 0;
    }
  }

  public void setSelected(long selected) {
    nodePublisher.set(selected);
  }

  public void setConeOrientation(boolean tipped) {
    coneTippedPublisher.set(tipped);
  }
}
