// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

/** Gripper subsystem hardware interface. */
public interface GripperIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class GripperIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GripperIOInputs inputs) {}

  /** Run the gripper open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode on the gripper. */
  public default void setBrakeMode(boolean enable) {}
}
