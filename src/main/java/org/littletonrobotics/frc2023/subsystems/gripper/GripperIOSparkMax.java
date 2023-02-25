// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.SparkMaxPeriodicFrameConfig;

public class GripperIOSparkMax implements GripperIO {
  private boolean invert = false;
  private double reduction = 1.0;
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public GripperIOSparkMax() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        invert = true;
        reduction = 5.23 * (40.0 / 18.0);
        motor = new CANSparkMax(13, MotorType.kBrushless);
        encoder = motor.getEncoder();
        break;
      default:
        throw new RuntimeException("Invalid robot for GripperIOSparkMax!");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }

    motor.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      SparkMaxPeriodicFrameConfig.configNotLeader(motor);

      motor.setInverted(invert);
      motor.setSmartCurrentLimit(40);
      motor.enableVoltageCompensation(12.0);
    }

    motor.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
