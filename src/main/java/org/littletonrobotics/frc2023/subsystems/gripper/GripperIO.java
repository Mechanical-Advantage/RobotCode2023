// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.gripper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Gripper subsystem hardware interface. */
public interface GripperIO {
  /** Contains all of the input data received from hardware. */
  public static class GripperIOInputs implements LoggableInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("PositionRad", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GripperIOInputs inputs) {
  }

  /** Run the roller open loop at the specified voltage. */
  public default void setVoltage(double volts) {
  }

  /** Enable or disable brake mode on the roller. */
  public default void setBrakeMode(boolean enable) {
  }
}