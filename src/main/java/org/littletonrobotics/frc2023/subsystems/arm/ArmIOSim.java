// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.frc2023.Constants;

public class ArmIOSim implements ArmIO {
  private boolean invert = false;
  private double appliedVolts = 0.0;

  private double afterEncoderReduction = 0.0;
  private double jKGMetersSquared = 0.0;
  private double armLengthMeters = 0.0;
  private double minAngleRads = -Math.PI / 5;
  private double maxAngleRads = Math.PI / 2;
  private double armMassKG = 0.0;
  private boolean simulateGravity = false;

  private DCMotor singleJointedArmMotor = DCMotor.getNEO(1);

  private SingleJointedArmSim singleJointedArmSim =
      new SingleJointedArmSim(
          singleJointedArmMotor,
          afterEncoderReduction,
          jKGMetersSquared,
          armLengthMeters,
          minAngleRads,
          maxAngleRads,
          armMassKG,
          simulateGravity);

  public ArmIOSim() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    singleJointedArmSim.update(Constants.loopPeriodSecs);

    inputs.positionRad = singleJointedArmSim.getAngleRads();
    inputs.velocityRadPerSec = singleJointedArmSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {singleJointedArmSim.getCurrentDrawAmps()};
    inputs.tempCelcius = new double[] {};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    singleJointedArmSim.setInputVoltage(appliedVolts);
  }
}
