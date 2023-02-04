// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intakes.cubeIntake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CubeIntakeIO {
  @AutoLog
  public static class CubeIntakeIOInputs {
    public double armAbsolutePosition = 0.0;
    public double armPositionRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};

    public double intakePositionRad = 0.0;
    public double inakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intakeCurrentAmps = new double[] {};
    public double[] intakeTempCelcius = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(CubeIntakeIOInputs inputs) {}

  /** Set the intake roller voltage */
  public default void setIntakeVoltage(double volts) {}

  /** Set the arm motor voltage */
  public default void setArmVoltage(double volts) {}
}
