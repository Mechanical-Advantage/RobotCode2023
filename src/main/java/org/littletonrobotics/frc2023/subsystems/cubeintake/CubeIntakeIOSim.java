// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.cubeintake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.frc2023.Constants;

public class CubeIntakeIOSim implements CubeIntakeIO {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(DCMotor.getNEO(1), 50, 0.5, 0.5, 0.0, Math.PI / 2.0, true);
  private double armAppliedVolts = 0.0;

  public CubeIntakeIOSim() {
    System.out.println("[Init] Creating CubeIntakeIOSim");
    armSim.setState(VecBuilder.fill(Math.PI / 2.0, 0.0));
  }

  @Override
  public void updateInputs(CubeIntakeIOInputs inputs) {
    armSim.update(Constants.loopPeriodSecs);

    inputs.armInternalPositionRad = armSim.getAngleRads();

    inputs.armInternalVelocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.armAppliedVolts = armAppliedVolts;
    inputs.armCurrentAmps = new double[] {armSim.getCurrentDrawAmps()};
    inputs.armTempCelcius = new double[] {};
  }

  @Override
  public void setArmVoltage(double voltage) {
    armAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    armSim.setInputVoltage(armAppliedVolts);
  }

  @Override
  public void setRollerVoltage(double voltage) {}
}
