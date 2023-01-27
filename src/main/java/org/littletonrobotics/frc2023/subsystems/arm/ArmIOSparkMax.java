// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax shoulderSparkMax;
  private final RelativeEncoder shoulderEncoder;
  private final AbsoluteEncoder shoulderAbsoluteEncoder;
  private final double shoulderReduction;
  private final boolean isShoulderMotorInverted;
  private final Rotation2d shoulderAbsoluteEncoderOffset;

  private final CANSparkMax elbowSparkMax;
  private final RelativeEncoder elbowEncoder;
  private final AbsoluteEncoder elbowAbsoluteEncoder;
  private final double elbowReduction;
  private final boolean isElbowMotorInverted;
  private final Rotation2d elbowAbsoluteEncoderOffset;

  private final CANSparkMax wristSparkMax;
  private final RelativeEncoder wristEncoder;
  private final AbsoluteEncoder wristAbsoluteEncoder;
  private final double wristReduction;
  private final boolean isWristMotorInverted;
  private final Rotation2d wristAbsoluteEncoderOffset;

  public ArmIOSparkMax(int index) {
    // Shoulder config
    shoulderSparkMax = new CANSparkMax(15, MotorType.kBrushless);
    shoulderEncoder = shoulderSparkMax.getEncoder();
    shoulderAbsoluteEncoder = shoulderSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderReduction = 1.0;
    isShoulderMotorInverted = false;
    shoulderAbsoluteEncoderOffset = new Rotation2d(0.0);

    // Elbow config
    elbowSparkMax = new CANSparkMax(16, MotorType.kBrushless);
    elbowEncoder = shoulderSparkMax.getEncoder();
    elbowAbsoluteEncoder = shoulderSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    elbowReduction = 1.0;
    isElbowMotorInverted = false;
    elbowAbsoluteEncoderOffset = new Rotation2d(0.0);

    // Wrist config
    wristSparkMax = new CANSparkMax(17, MotorType.kBrushless);
    wristEncoder = shoulderSparkMax.getEncoder();
    wristAbsoluteEncoder = shoulderSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    wristReduction = 1.0;
    isWristMotorInverted = false;
    wristAbsoluteEncoderOffset = new Rotation2d(0.0);

    // Set remaining config
    if (SparkMaxBurnManager.shouldBurn()) {
      shoulderSparkMax.restoreFactoryDefaults();
      elbowSparkMax.restoreFactoryDefaults();
      wristSparkMax.restoreFactoryDefaults();
    }

    shoulderSparkMax.setInverted(isShoulderMotorInverted);
    elbowSparkMax.setInverted(isElbowMotorInverted);
    wristSparkMax.setInverted(isWristMotorInverted);

    shoulderSparkMax.setSmartCurrentLimit(30);
    elbowSparkMax.setSmartCurrentLimit(30);
    wristSparkMax.setSmartCurrentLimit(30);

    shoulderSparkMax.enableVoltageCompensation(12.0);
    elbowSparkMax.enableVoltageCompensation(12.0);
    wristSparkMax.enableVoltageCompensation(12.0);

    shoulderEncoder.setPosition(0.0);
    elbowEncoder.setPosition(0.0);
    wristEncoder.setPosition(0.0);

    shoulderSparkMax.setCANTimeout(0);
    elbowSparkMax.setCANTimeout(0);
    wristSparkMax.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      shoulderSparkMax.burnFlash();
      elbowSparkMax.burnFlash();
      wristSparkMax.burnFlash();
    }
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.shoulderAbsolutePositionRad =
        new Rotation2d(shoulderAbsoluteEncoder.getPosition())
            .minus(shoulderAbsoluteEncoderOffset)
            .getRadians();
    inputs.shoulderPositionRad =
        Units.rotationsToRadians(shoulderEncoder.getPosition()) / shoulderReduction;
    inputs.shoulderVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shoulderEncoder.getVelocity())
            / shoulderReduction;
    inputs.shoulderAppliedVolts =
        shoulderSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.shoulderCurrentAmps = new double[] {shoulderSparkMax.getOutputCurrent()};
    inputs.shoulderTempCelcius = new double[] {shoulderSparkMax.getMotorTemperature()};

    inputs.elbowAbsolutePositionRad =
        new Rotation2d(elbowAbsoluteEncoder.getPosition())
            .minus(elbowAbsoluteEncoderOffset)
            .getRadians();
    inputs.elbowPositionRad = Units.rotationsToRadians(elbowEncoder.getPosition()) / elbowReduction;
    inputs.elbowVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(elbowEncoder.getVelocity()) / elbowReduction;
    inputs.elbowAppliedVolts =
        elbowSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.elbowCurrentAmps = new double[] {elbowSparkMax.getOutputCurrent()};
    inputs.elbowTempCelcius = new double[] {elbowSparkMax.getMotorTemperature()};

    inputs.wristAbsolutePositionRad =
        new Rotation2d(wristAbsoluteEncoder.getPosition())
            .minus(wristAbsoluteEncoderOffset)
            .getRadians();
    inputs.wristPositionRad = Units.rotationsToRadians(wristEncoder.getPosition()) / wristReduction;
    inputs.wristVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristEncoder.getVelocity()) / wristReduction;
    inputs.wristAppliedVolts =
        wristSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.wristCurrentAmps = new double[] {wristSparkMax.getOutputCurrent()};
    inputs.wristTempCelcius = new double[] {wristSparkMax.getMotorTemperature()};
  }

  public void setVoltage(double shoulderVolts, double elbowVolts, double wristVolts) {
    shoulderSparkMax.setVoltage(shoulderVolts);
    elbowSparkMax.setVoltage(elbowVolts);
    wristSparkMax.setVoltage(wristVolts);
  }

  public void setBrakeMode(boolean shoulderBrake, boolean elbowBrake, boolean wristBrake) {
    shoulderSparkMax.setIdleMode(shoulderBrake ? IdleMode.kBrake : IdleMode.kCoast);
    elbowSparkMax.setIdleMode(elbowBrake ? IdleMode.kBrake : IdleMode.kCoast);
    wristSparkMax.setIdleMode(wristBrake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
