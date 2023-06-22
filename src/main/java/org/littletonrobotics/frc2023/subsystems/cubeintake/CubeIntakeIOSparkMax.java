// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.cubeintake;

import static org.littletonrobotics.frc2023.util.CleanSparkMaxValue.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.SparkMaxPeriodicFrameConfig;

public class CubeIntakeIOSparkMax implements CubeIntakeIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax rollerSparkMax;
  private final RelativeEncoder armInternalEncoder;
  private final boolean armInvert;
  private final boolean rollerInvert;
  private final double armInternalEncoderReduction;
  private final double rollerInternalEncoderReduction;
  private final RelativeEncoder rollerInternalEncoder;

  public CubeIntakeIOSparkMax() {
    System.out.println("[Init] Creating CubeIntakeIOSparkMax");

    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        armSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        rollerSparkMax = new CANSparkMax(12, MotorType.kBrushless);

        armInvert = false;
        rollerInvert = true;
        armInternalEncoderReduction = 5.0 * 5.0 * (30.0 / 18.0) * (30.0 / 18.0);
        rollerInternalEncoderReduction = 5.0 * 10.0;

        break;
      default:
        throw new RuntimeException("Invalid robot for CubeIntakeIOSparkMax!");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.restoreFactoryDefaults();
      rollerSparkMax.restoreFactoryDefaults();
    }

    armSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    rollerSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    armInternalEncoder = armSparkMax.getEncoder();
    rollerInternalEncoder = rollerSparkMax.getEncoder();

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      SparkMaxPeriodicFrameConfig.configNotLeader(armSparkMax);
      SparkMaxPeriodicFrameConfig.configNotLeader(rollerSparkMax);

      armInternalEncoder.setPosition(0.0);
      armInternalEncoder.setMeasurementPeriod(10);
      armInternalEncoder.setAverageDepth(2);

      rollerInternalEncoder.setPosition(0.0);
      rollerInternalEncoder.setMeasurementPeriod(10);
      rollerInternalEncoder.setAverageDepth(2);

      armSparkMax.setInverted(armInvert);
      rollerSparkMax.setInverted(rollerInvert);

      armSparkMax.setSmartCurrentLimit(30);
      rollerSparkMax.setSmartCurrentLimit(30);

      armSparkMax.enableVoltageCompensation(12.0);
      rollerSparkMax.enableVoltageCompensation(12.0);
    }

    armSparkMax.setCANTimeout(0);
    rollerSparkMax.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
      rollerSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(CubeIntakeIOInputs inputs) {
    inputs.armInternalPositionRad =
        cleanSparkMaxValue(
            inputs.armInternalPositionRad,
            Units.rotationsToRadians(armInternalEncoder.getPosition())
                / armInternalEncoderReduction);
    inputs.armInternalVelocityRadPerSec =
        cleanSparkMaxValue(
            inputs.armInternalVelocityRadPerSec,
            Units.rotationsPerMinuteToRadiansPerSecond(armInternalEncoder.getPosition())
                / armInternalEncoderReduction);
    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * armSparkMax.getBusVoltage();
    inputs.armCurrentAmps = new double[] {armSparkMax.getOutputCurrent()};
    inputs.armTempCelcius = new double[] {armSparkMax.getMotorTemperature()};

    inputs.rollerAppliedVolts = rollerSparkMax.getAppliedOutput() * rollerSparkMax.getBusVoltage();
    inputs.rollerCurrentAmps = new double[] {rollerSparkMax.getOutputCurrent()};

    inputs.rollerTempCelcius = new double[] {rollerSparkMax.getMotorTemperature()};
    inputs.rollerInternalPositionRad =
        cleanSparkMaxValue(
            inputs.rollerInternalPositionRad,
            Units.rotationsToRadians(rollerInternalEncoder.getPosition())
                / rollerInternalEncoderReduction);
    inputs.rollerInternalVelocityRadPerSec =
        cleanSparkMaxValue(
            inputs.rollerInternalVelocityRadPerSec,
            Units.rotationsPerMinuteToRadiansPerSecond(rollerInternalEncoder.getPosition())
                / rollerInternalEncoderReduction);
  }

  @Override
  public void setArmVoltage(double voltage) {
    armSparkMax.setVoltage(voltage);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerSparkMax.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean armBrake, boolean rollerBrake) {
    armSparkMax.setIdleMode(armBrake ? IdleMode.kBrake : IdleMode.kCoast);
    rollerSparkMax.setIdleMode(rollerBrake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
