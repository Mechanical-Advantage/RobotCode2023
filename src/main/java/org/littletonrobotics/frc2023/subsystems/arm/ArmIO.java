// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double shoulderAbsolutePositionRad = 0.0;
    public double shoulderPositionRad = 0.0;
    public double shoulderVelocityRadPerSec = 0.0;
    public double shoulderAppliedVolts = 0.0;
    public double[] shoulderCurrentAmps = new double[] {};
    public double[] shoulderTempCelcius = new double[] {};

    public double elbowAbsolutePositionRad = 0.0;
    public double elbowPositionRad = 0.0;
    public double elbowVelocityRadPerSec = 0.0;
    public double elbowAppliedVolts = 0.0;
    public double[] elbowCurrentAmps = new double[] {};
    public double[] elbowTempCelcius = new double[] {};

    public double wristAbsolutePositionRad = 0.0;
    public double wristPositionRad = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double[] wristCurrentAmps = new double[] {};
    public double[] wristTempCelcius = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the motors at the specified voltages. */
  public default void setVoltage(double shoulderVolts, double elbowVolts, double wristVolts) {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean shoulderBrake, boolean elbowBrake, boolean wristBrake) {}
}
