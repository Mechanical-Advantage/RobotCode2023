// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.cubeintake;

import org.littletonrobotics.junction.AutoLog;

public interface CubeIntakeIO {
  @AutoLog
  public static class CubeIntakeIOInputs {

    public double armInternalPositionRad = 0.0;
    public double armInternalVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};

    public double rollerAppliedVolts = 0.0;
    public double[] rollerCurrentAmps = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(CubeIntakeIOInputs inputs) {}

  /** Set the arm motor voltage */
  public default void setArmVoltage(double volts) {}

  /** Set the intake roller voltage */
  public default void setRollerVoltage(double volts) {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean armBrake, boolean rollerBrake) {}
}
