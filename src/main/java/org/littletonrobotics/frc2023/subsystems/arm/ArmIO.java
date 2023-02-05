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

  /** Sets the arm config, must be called before other methods. */
  public default void setConfig(ArmConfig config) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the shoulder motor(s) at the specified voltages. */
  public default void setShoulderVoltage(double volts) {}

  /** Run the elbow motor(s) at the specified voltages. */
  public default void setElbowVoltage(double volts) {}

  /** Run the wrist motor(s) at the specified voltages. */
  public default void setWristVoltage(double volts) {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean shoulderBrake, boolean elbowBrake, boolean wristBrake) {}
}
