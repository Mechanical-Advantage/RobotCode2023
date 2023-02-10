// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.objectivetracker;

import org.littletonrobotics.junction.AutoLog;

public interface GridSelectorIO {
  @AutoLog
  public static class GridSelectorIOInputs {
    public long selected = -1;
  }

  public default void updateInputs(GridSelectorIOInputs inputs) {}

  public default void setSelected(long selected) {}
}
