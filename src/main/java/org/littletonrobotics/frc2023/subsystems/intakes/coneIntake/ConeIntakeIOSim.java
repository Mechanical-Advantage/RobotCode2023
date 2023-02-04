// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intakes.coneIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.frc2023.Constants;

/** Add your docs here. */
public class ConeIntakeIOSim implements ConeIntakeIO {
  private double armAppliedVolts = 0.0;

  private double afterEncoderReduction = 100.0;
  private double jKGMetersSquared = 1.0;
  private double armLengthMeters = 1.0;
  private double minAngleRads = -Math.PI / 5;
  private double maxAngleRads = Math.PI / 2;
  private double armMassKG = 1.0;
  private boolean simulateGravity = false;

  private DCMotor armMotor = DCMotor.getNEO(1);

  private SingleJointedArmSim singleJointedArmSim =
      new SingleJointedArmSim(
          armMotor,
          afterEncoderReduction,
          jKGMetersSquared,
          armLengthMeters,
          minAngleRads,
          maxAngleRads,
          armMassKG,
          simulateGravity);

  public ConeIntakeIOSim() {}

  @Override
  public void updateInputs(ConeIntakeIOInputs inputs) {
    singleJointedArmSim.update(Constants.loopPeriodSecs);

    inputs.armPositionRad = singleJointedArmSim.getAngleRads();
    inputs.armVelocityRadPerSec = singleJointedArmSim.getVelocityRadPerSec();
    inputs.armAppliedVolts = armAppliedVolts;
    inputs.armCurrentAmps = new double[] {singleJointedArmSim.getCurrentDrawAmps()};
    inputs.armTempCelcius = new double[] {};
  }

  @Override
  public void setArmVoltage(double voltage) {
    armAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    singleJointedArmSim.setInputVoltage(voltage);
  }

  @Override
  public void setIntakeVoltage(double voltage) {}
}
