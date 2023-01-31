// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class GripperIOSparkMax implements GripperIO {
  private boolean invert = false;

  private double afterEncoderReduction;
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public GripperIOSparkMax() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        motor = new CANSparkMax(4, MotorType.kBrushless);
        afterEncoderReduction = 60.0 / 16.0;
        invert = true;

        encoder = motor.getEncoder();
        break;
      default:
        throw new RuntimeException("Invalid robot for GripperIOSparkMax!");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }

    motor.setInverted(invert);
    motor.setSmartCurrentLimit(30);
    motor.enableVoltageCompensation(12.0);

    motor.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
      inputs.positionRad =
        Units.rotationsToRadians(encoder.getPosition()) / afterEncoderReduction;
      inputs.velocityRadPerSec =
          Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
              / afterEncoderReduction;
      inputs.appliedVolts =
          motor.getAppliedOutput() * RobotController.getBatteryVoltage();
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