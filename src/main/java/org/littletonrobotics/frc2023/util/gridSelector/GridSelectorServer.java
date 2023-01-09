// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.gridSelector;

import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import io.javalin.websocket.WsContext;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class GridSelectorServer {
  private Map<String, WsContext> clients = new HashMap<>();
  private int selected = -1;

  public GridSelectorServer() {
    var app =
        Javalin.create(
            config -> {
              config.staticFiles.add(
                  Paths.get(
                          Filesystem.getDeployDirectory().getAbsolutePath().toString(),
                          "gridSelector")
                      .toString(),
                  Location.EXTERNAL);
            });

    app.ws(
        "/ws",
        ws -> {
          ws.onConnect(
              context -> {
                System.out.println("[GridSelectorServer] Connected to " + context.host());
                clients.put(context.getSessionId(), context);
                context.send(Integer.toString(selected));
              });
          ws.onClose(
              context -> {
                System.out.println("[GridSelectorServer] Disconnected from " + context.host());
                clients.remove(context.getSessionId());
              });
          ws.onMessage(
              context -> {
                selected = Integer.parseInt(context.message());
                updateClients();
              });
        });

    app.start(5800);
  }

  private void updateClients() {
    clients
        .values()
        .forEach(
            (context) -> {
              context.send(Integer.toString(selected));
            });
  }

  public void setSelected(int index) {
    selected = index;
    updateClients();
  }

  public Optional<Integer> getSelected() {
    if (selected == -1) {
      return Optional.empty();
    } else {
      return Optional.of(selected);
    }
  }
}
