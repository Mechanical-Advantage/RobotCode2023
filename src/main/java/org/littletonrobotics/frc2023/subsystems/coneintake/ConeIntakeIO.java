// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.coneintake;

import org.littletonrobotics.junction.AutoLog;

public interface ConeIntakeIO {
  @AutoLog
  public static class ConeIntakeIOInputs {
    public double armAbsolutePositionRad = 0.0;
    public double armRelativePositionRad = 0.0;
    public double armInternalPositionRad = 0.0;
    public double armRelativeVelocityRadPerSec = 0.0;
    public double armInternalVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};

    public double rollerAppliedVolts = 0.0;
    public double[] rollerCurrentAmps = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ConeIntakeIOInputs inputs) {}

  /** Set the arm motor voltage */
  public default void setArmVoltage(double volts) {}

  /** Set the intake roller voltage */
  public default void setRollerVoltage(double volts) {}
}
