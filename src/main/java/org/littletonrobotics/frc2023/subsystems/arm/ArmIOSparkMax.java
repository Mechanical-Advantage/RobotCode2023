// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class ArmIOSparkMax implements ArmIO {
  private boolean invert = false;

  private double afterEncoderReduction = 1.0;
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public ArmIOSparkMax() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        motor = new CANSparkMax(4, MotorType.kBrushless);
        afterEncoderReduction = 60.0 / 16.0;
        invert = true;

        encoder = motor.getEncoder();
        break;
      default:
        throw new RuntimeException("Invalid robot for IntakeIOSparkMax!");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }

    motor.setInverted(invert);
    motor.setSmartCurrentLimit(30);
    motor.enableVoltageCompensation(12.0);

    motor.setCANTimeout(0);

    boolean enable = true;
    motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);

    if (SparkMaxBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / afterEncoderReduction;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / afterEncoderReduction;
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.tempCelcius = new double[] {motor.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
