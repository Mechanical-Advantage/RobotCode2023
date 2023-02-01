// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intakes.coneIntake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ConeIntakeIO {
  @AutoLog
  public static class ConeIntakeIOInputs {
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
  public default void updateInputs(ConeIntakeIOInputs inputs) {}
  
  /** Set the intake roller voltage */
  public default void setIntakeVoltage(double volts) {}
  
  /** Set the arm motor voltage */
  public default void setArmVoltage(double volts) {}
}
