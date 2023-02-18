// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents a subsystem unit that requirse a periodic callback but not require a hardware mutex.
 */
public abstract class VirtualSubsystem extends SubsystemBase {
  private static List<VirtualSubsystem> subsystems = new ArrayList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  /** Calls {@link #periodic()} on all virtual subsystems. */
  public static void periodicAll() {
    for (var subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  /** This method is called periodically once per loop cycle. */
  public abstract void periodic();
}
