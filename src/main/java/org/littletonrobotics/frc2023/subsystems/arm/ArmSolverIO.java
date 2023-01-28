// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import org.littletonrobotics.frc2023.util.arm.ArmTrajectory;
import org.littletonrobotics.junction.AutoLog;

public interface ArmSolverIO {
  @AutoLog
  public static class ArmSolverIOInputs {
    public long parameterHash = 0;
    public double totalTime = 0.0;
    public double[] shoulderPoints = new double[] {};
    public double[] elbowPoints = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmSolverIOInputs inputs) {}

  /** Sets the arm config data to be used when generating paths. */
  public default void setConfig(String configJson) {}

  /** Requests a trajectory to generate. */
  public default void request(ArmTrajectory.Parameters parameters) {}
}
