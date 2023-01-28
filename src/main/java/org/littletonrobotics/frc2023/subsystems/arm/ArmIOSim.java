// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.arm.ArmConfig;

public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim shoulderSim;
  private SingleJointedArmSim elbowSim;
  private SingleJointedArmSim wristSim;

  private double shoulderAppliedVolts = 0.0;
  private double elbowAppliedVolts = 0.0;
  private double wristAppliedVolts = 0.0;

  public ArmIOSim() {}

  public void setConfig(ArmConfig config) {
    shoulderSim =
        new SingleJointedArmSim(
            config.shoulder().motor(),
            1.0,
            config.shoulder().moi(),
            config.shoulder().length(),
            config.shoulder().minAngle(),
            config.shoulder().maxAngle(),
            config.shoulder().mass(),
            true);
    elbowSim =
        new SingleJointedArmSim(
            config.elbow().motor(),
            1.0,
            config.elbow().moi(),
            config.elbow().length(),
            config.elbow().minAngle(),
            config.elbow().maxAngle(),
            config.elbow().mass(),
            false);
    wristSim =
        new SingleJointedArmSim(
            config.wrist().motor(),
            1.0,
            config.wrist().moi(),
            config.wrist().length(),
            config.wrist().minAngle(),
            config.wrist().maxAngle(),
            config.wrist().mass(),
            false);
  }

  public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setVoltage(0.0, 0.0, 0.0);
    }

    shoulderSim.update(Constants.loopPeriodSecs);
    elbowSim.update(Constants.loopPeriodSecs);
    wristSim.update(Constants.loopPeriodSecs);

    inputs.shoulderAbsolutePositionRad = shoulderSim.getAngleRads();
    inputs.shoulderPositionRad = shoulderSim.getAngleRads();
    inputs.shoulderVelocityRadPerSec = shoulderSim.getVelocityRadPerSec();
    inputs.shoulderAppliedVolts = shoulderAppliedVolts;
    inputs.shoulderCurrentAmps = new double[] {shoulderSim.getCurrentDrawAmps()};
    inputs.shoulderTempCelcius = new double[] {};

    inputs.elbowAbsolutePositionRad = elbowSim.getAngleRads();
    inputs.elbowPositionRad = elbowSim.getAngleRads();
    inputs.elbowVelocityRadPerSec = elbowSim.getVelocityRadPerSec();
    inputs.elbowAppliedVolts = elbowAppliedVolts;
    inputs.elbowCurrentAmps = new double[] {elbowSim.getCurrentDrawAmps()};
    inputs.elbowTempCelcius = new double[] {};

    inputs.wristAbsolutePositionRad = wristSim.getAngleRads();
    inputs.wristPositionRad = wristSim.getAngleRads();
    inputs.wristVelocityRadPerSec = wristSim.getVelocityRadPerSec();
    inputs.wristAppliedVolts = wristAppliedVolts;
    inputs.wristCurrentAmps = new double[] {wristSim.getCurrentDrawAmps()};
    inputs.wristTempCelcius = new double[] {};
  }

  public void setVoltage(double shoulderVolts, double elbowVolts, double wristVolts) {
    shoulderSim.setInputVoltage(shoulderVolts);
    elbowSim.setInputVoltage(elbowVolts);
    wristSim.setInputVoltage(wristVolts);
    shoulderAppliedVolts = shoulderVolts;
    elbowAppliedVolts = elbowVolts;
    wristAppliedVolts = wristVolts;
  }
}
