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
  private boolean lastEnabled = false;

  private double shoulderAppliedVolts = 0.0;
  private double elbowAppliedVolts = 0.0;
  private double wristAppliedVolts = 0.0;

  public ArmIOSim() {}

  public void setConfig(ArmConfig config) {
    this.config = config;
    dynamics = new ArmDynamics(config);
    wristSim =
        new SingleJointedArmSim(
            config.wrist().motor().physics(),
            1.0, // Reduction is included in the motor
            config.wrist().moi(),
            config.wrist().length(),
            config.wrist().minAngle(),
            config.wrist().maxAngle(),
            false);
    wristSim.setState(VecBuilder.fill(-Math.PI / 2.0, 0.0));
  }

  public void updateInputs(ArmIOInputs inputs) {
    // Reset voltages when disabled
    if (DriverStation.isDisabled()) {
      setShoulderVoltage(0.0);
      setElbowVoltage(0.0);
      setWristVoltage(0.0);
    }

    // Reset position on enable
    if (DriverStation.isEnabled() && !lastEnabled) {
      shoulderElbowStates = VecBuilder.fill(Math.PI / 2.0, Math.PI, 0.0, 0.0);
      wristSim.setState(VecBuilder.fill(-Math.PI / 2.0, 0.0));
    }
    lastEnabled = DriverStation.isEnabled();

    // Update sim states
    shoulderElbowStates =
        dynamics.simulate(
            shoulderElbowStates,
            VecBuilder.fill(shoulderAppliedVolts, elbowAppliedVolts),
            Constants.loopPeriodSecs);
    wristSim.update(Constants.loopPeriodSecs);

    // Log sim data
    inputs.shoulderAbsolutePositionRad = shoulderElbowStates.get(0, 0);
    inputs.shoulderRelativePositionRad = shoulderElbowStates.get(0, 0);
    inputs.shoulderInternalPositionRad = shoulderElbowStates.get(0, 0);
    inputs.shoulderRelativeVelocityRadPerSec = shoulderElbowStates.get(2, 0);
    inputs.shoulderInternalVelocityRadPerSec = shoulderElbowStates.get(2, 0);
    inputs.shoulderAppliedVolts = shoulderAppliedVolts;
    inputs.shoulderCurrentAmps =
        new double[] {
          config
              .shoulder()
              .motor()
              .physics()
              .getCurrent(shoulderElbowStates.get(2, 0), shoulderAppliedVolts)
        };
    inputs.shoulderTempCelcius = new double[] {};
    inputs.elbowAbsolutePositionRad = shoulderElbowStates.get(1, 0);
    inputs.elbowRelativePositionRad = shoulderElbowStates.get(1, 0);
    inputs.elbowInternalPositionRad = shoulderElbowStates.get(1, 0);
    inputs.elbowRelativeVelocityRadPerSec = shoulderElbowStates.get(3, 0);
    inputs.elbowInternalVelocityRadPerSec = shoulderElbowStates.get(3, 0);
    inputs.elbowAppliedVolts = elbowAppliedVolts;
    inputs.elbowCurrentAmps =
        new double[] {
          config
              .elbow()
              .motor()
              .physics()
              .getCurrent(shoulderElbowStates.get(3, 0), elbowAppliedVolts)
        };
    inputs.elbowTempCelcius = new double[] {};
    inputs.wristAbsolutePositionRad = wristSim.getAngleRads();
    inputs.wristRelativePositionRad = wristSim.getAngleRads();
    inputs.wristInternalPositionRad = wristSim.getAngleRads();
    inputs.wristRelativeVelocityRadPerSec = wristSim.getVelocityRadPerSec();
    inputs.wristInternalVelocityRadPerSec = wristSim.getVelocityRadPerSec();
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
