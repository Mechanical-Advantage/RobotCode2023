// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.objectivetracker;

import org.littletonrobotics.junction.AutoLog;

public interface NodeSelectorIO {
  @AutoLog
  public static class NodeSelectorIOInputs {
    public long selectedNode = -1;
    public long coneTipped = -1;
  }

  public default void updateInputs(NodeSelectorIOInputs inputs) {}

  public default void setSelected(long selected) {}

  public default void setConeOrientation(boolean tipped) {}
}
