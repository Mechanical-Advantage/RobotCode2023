// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.frc2023.Constants;

public class ArmIOSim implements ArmIO {
  private ArmConfig config;
  private ArmDynamics dynamics;
  private Vector<N4> shoulderElbowStates = VecBuilder.fill(Math.PI / 2.0, Math.PI, 0.0, 0.0);
  private SingleJointedArmSim wristSim;

  private double shoulderAppliedVolts = 0.0;
  private double elbowAppliedVolts = 0.0;
  private double wristAppliedVolts = 0.0;

  public ArmIOSim() {}

  public void setConfig(ArmConfig config) {
    this.config = config;
    dynamics = new ArmDynamics(config);
    wristSim =
        new SingleJointedArmSim(
            config.wrist().motor(),
            1.0, // Reduction is included in the motor
            config.wrist().moi(),
            config.wrist().length(),
            config.wrist().minAngle(),
            config.wrist().maxAngle(),
            config.wrist().mass(),
            false);
    wristSim.setState(VecBuilder.fill(0.0, 0.0));
  }

  public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setShoulderVoltage(0.0);
      setElbowVoltage(0.0);
      setWristVoltage(0.0);
    }

    shoulderElbowStates =
        dynamics.simulate(
            shoulderElbowStates,
            VecBuilder.fill(shoulderAppliedVolts, elbowAppliedVolts),
            Constants.loopPeriodSecs);
    wristSim.update(Constants.loopPeriodSecs);

    inputs.shoulderAbsolutePositionRad = shoulderElbowStates.get(0, 0);
    inputs.shoulderPositionRad = shoulderElbowStates.get(0, 0);
    inputs.shoulderVelocityRadPerSec = shoulderElbowStates.get(2, 0);
    inputs.shoulderAppliedVolts = shoulderAppliedVolts;
    inputs.shoulderCurrentAmps =
        new double[] {
          config.shoulder().motor().getCurrent(shoulderElbowStates.get(2, 0), shoulderAppliedVolts)
        };
    inputs.shoulderTempCelcius = new double[] {};

    inputs.elbowAbsolutePositionRad = shoulderElbowStates.get(1, 0);
    inputs.elbowPositionRad = shoulderElbowStates.get(1, 0);
    inputs.elbowVelocityRadPerSec = shoulderElbowStates.get(3, 0);
    inputs.elbowAppliedVolts = elbowAppliedVolts;
    inputs.elbowCurrentAmps =
        new double[] {
          config.elbow().motor().getCurrent(shoulderElbowStates.get(3, 0), elbowAppliedVolts)
        };
    inputs.elbowTempCelcius = new double[] {};

    inputs.wristAbsolutePositionRad = wristSim.getAngleRads();
    inputs.wristPositionRad = wristSim.getAngleRads();
    inputs.wristVelocityRadPerSec = wristSim.getVelocityRadPerSec();
    inputs.wristAppliedVolts = wristAppliedVolts;
    inputs.wristCurrentAmps = new double[] {wristSim.getCurrentDrawAmps()};
    inputs.wristTempCelcius = new double[] {};
  }

  public void setShoulderVoltage(double volts) {
    shoulderAppliedVolts = MathUtil.clamp(volts, -12, 12);
  }

  public void setElbowVoltage(double volts) {
    elbowAppliedVolts = MathUtil.clamp(volts, -12, 12);
  }

  public void setWristVoltage(double volts) {
    wristAppliedVolts = MathUtil.clamp(volts, -12, 12);
    wristSim.setInputVoltage(wristAppliedVolts);
  }
}
