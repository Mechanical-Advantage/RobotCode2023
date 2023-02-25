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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.SparkMaxPeriodicFrameConfig;

public class ArmIOSparkMax implements ArmIO {
  private ArmConfig config;

  private final CANSparkMax shoulderSparkMax;
  private final CANSparkMax shoulderSparkMaxFollower;
  private final DutyCycleEncoder shoulderAbsoluteEncoder;
  private final Encoder shoulderRelativeEncoder;
  private final RelativeEncoder shoulderInternalEncoder;
  private final boolean isShoulderMotorInverted;
  private final boolean isShoulderExternalEncoderInverted;
  private final Rotation2d shoulderAbsoluteEncoderOffset;

  private final CANSparkMax elbowSparkMax;
  private final DutyCycleEncoder elbowAbsoluteEncoder;
  private final Encoder elbowRelativeEncoder;
  private final RelativeEncoder elbowInternalEncoder;
  private final boolean isElbowMotorInverted;
  private final boolean isElbowExternalEncoderInverted;
  private final Rotation2d elbowAbsoluteEncoderOffset;

  private final CANSparkMax wristSparkMax;
  private final DutyCycleEncoder wristAbsoluteEncoder;
  private final Encoder wristRelativeEncoder;
  private final RelativeEncoder wristInternalEncoder;
  private final boolean isWristMotorInverted;
  private final boolean isWristExternalEncoderInverted;
  private final Rotation2d wristAbsoluteEncoderOffset;

  public ArmIOSparkMax() {
    // Shoulder config
    shoulderSparkMax = new CANSparkMax(4, MotorType.kBrushless);
    shoulderSparkMaxFollower = new CANSparkMax(5, MotorType.kBrushless);
    shoulderAbsoluteEncoder = new DutyCycleEncoder(0);
    shoulderAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    shoulderRelativeEncoder = new Encoder(2, 1, false);
    isShoulderMotorInverted = false;
    isShoulderExternalEncoderInverted = false;
    shoulderAbsoluteEncoderOffset =
        new Rotation2d(-3.2918873072).plus(Rotation2d.fromDegrees(-90.0));
    shoulderRelativeEncoder.setDistancePerPulse((2 * Math.PI) / 2048);

    // Elbow config
    elbowSparkMax = new CANSparkMax(15, MotorType.kBrushless);
    elbowAbsoluteEncoder = new DutyCycleEncoder(3);
    elbowAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    elbowRelativeEncoder = new Encoder(5, 4, false);
    isElbowMotorInverted = false;
    isElbowExternalEncoderInverted = true;
    elbowAbsoluteEncoderOffset = new Rotation2d(-4.6001713072).plus(Rotation2d.fromDegrees(180.0));
    elbowRelativeEncoder.setDistancePerPulse((2 * Math.PI) / 2048);

    // Wrist config
    wristSparkMax = new CANSparkMax(14, MotorType.kBrushless);
    wristAbsoluteEncoder = new DutyCycleEncoder(6);
    wristAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    wristRelativeEncoder = new Encoder(8, 7, false);
    isWristMotorInverted = false;
    isWristExternalEncoderInverted = true;
    wristAbsoluteEncoderOffset = new Rotation2d(-1.977401);
    wristRelativeEncoder.setDistancePerPulse((2 * Math.PI) / 2048);

    // Set remaining config
    if (SparkMaxBurnManager.shouldBurn()) {
      shoulderSparkMax.restoreFactoryDefaults();
      shoulderSparkMaxFollower.restoreFactoryDefaults();
      elbowSparkMax.restoreFactoryDefaults();
      wristSparkMax.restoreFactoryDefaults();
    }

    shoulderSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    shoulderSparkMaxFollower.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    elbowSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    wristSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    shoulderInternalEncoder = shoulderSparkMax.getEncoder();
    elbowInternalEncoder = elbowSparkMax.getEncoder();
    wristInternalEncoder = wristSparkMax.getEncoder();

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      SparkMaxPeriodicFrameConfig.configLeaderFollower(shoulderSparkMax);
      SparkMaxPeriodicFrameConfig.configLeaderFollower(shoulderSparkMaxFollower);
      SparkMaxPeriodicFrameConfig.configNotLeader(elbowSparkMax);
      SparkMaxPeriodicFrameConfig.configNotLeader(wristSparkMax);

      shoulderSparkMaxFollower.follow(shoulderSparkMax);
      shoulderSparkMax.setInverted(isShoulderMotorInverted);
      elbowSparkMax.setInverted(isElbowMotorInverted);
      wristSparkMax.setInverted(isWristMotorInverted);

      shoulderSparkMax.setSmartCurrentLimit(40);
      shoulderSparkMaxFollower.setSmartCurrentLimit(40);
      elbowSparkMax.setSmartCurrentLimit(40);
      wristSparkMax.setSmartCurrentLimit(40);

      shoulderSparkMax.enableVoltageCompensation(12.0);
      elbowSparkMax.enableVoltageCompensation(12.0);
      wristSparkMax.enableVoltageCompensation(12.0);

      shoulderInternalEncoder.setMeasurementPeriod(10);
      shoulderInternalEncoder.setAverageDepth(2);
      shoulderInternalEncoder.setPosition(0.0);

      elbowInternalEncoder.setMeasurementPeriod(10);
      elbowInternalEncoder.setAverageDepth(2);
      elbowInternalEncoder.setPosition(0.0);

      wristInternalEncoder.setMeasurementPeriod(10);
      wristInternalEncoder.setAverageDepth(2);
      wristInternalEncoder.setPosition(0.0);
    }

    shoulderSparkMax.setCANTimeout(0);
    shoulderSparkMaxFollower.setCANTimeout(0);
    elbowSparkMax.setCANTimeout(0);
    wristSparkMax.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      shoulderSparkMax.burnFlash();
      elbowSparkMax.burnFlash();
      wristSparkMax.burnFlash();
    }
  }

  public void setConfig(ArmConfig config) {
    this.config = config;
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.shoulderAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(shoulderAbsoluteEncoder.get())
                    * (isShoulderExternalEncoderInverted ? -1 : 1)
                - shoulderAbsoluteEncoderOffset.getRadians());
    inputs.shoulderRelativePositionRad =
        shoulderRelativeEncoder.getDistance() * (isShoulderExternalEncoderInverted ? -1 : 1);
    inputs.shoulderInternalPositionRad =
        Units.rotationsToRadians(shoulderInternalEncoder.getPosition())
            / config.shoulder().motor().reduction();
    inputs.shoulderRelativeVelocityRadPerSec =
        shoulderRelativeEncoder.getRate() * (isShoulderExternalEncoderInverted ? -1 : 1);
    inputs.shoulderInternalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shoulderInternalEncoder.getPosition())
            / config.shoulder().motor().reduction();
    inputs.shoulderAppliedVolts =
        shoulderSparkMax.getAppliedOutput() * shoulderSparkMax.getBusVoltage();
    inputs.shoulderCurrentAmps =
        new double[] {
          shoulderSparkMax.getOutputCurrent(), shoulderSparkMaxFollower.getOutputCurrent()
        };
    inputs.shoulderTempCelcius =
        new double[] {
          shoulderSparkMax.getMotorTemperature(), shoulderSparkMaxFollower.getMotorTemperature()
        };

    inputs.elbowAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(elbowAbsoluteEncoder.get())
                    * (isElbowExternalEncoderInverted ? -1 : 1)
                - elbowAbsoluteEncoderOffset.getRadians());
    inputs.elbowRelativePositionRad =
        elbowRelativeEncoder.getDistance() * (isElbowExternalEncoderInverted ? -1 : 1);
    inputs.elbowInternalPositionRad =
        Units.rotationsToRadians(elbowInternalEncoder.getPosition())
            / config.elbow().motor().reduction();
    inputs.elbowRelativeVelocityRadPerSec =
        elbowRelativeEncoder.getRate() * (isElbowExternalEncoderInverted ? -1 : 1);
    inputs.elbowInternalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(elbowInternalEncoder.getPosition())
            / config.elbow().motor().reduction();
    inputs.elbowAppliedVolts = elbowSparkMax.getAppliedOutput() * elbowSparkMax.getBusVoltage();
    inputs.elbowCurrentAmps = new double[] {elbowSparkMax.getOutputCurrent()};
    inputs.elbowTempCelcius = new double[] {elbowSparkMax.getMotorTemperature()};

    inputs.wristAbsolutePositionRad =
        MathUtil.angleModulus(
            Units.rotationsToRadians(wristAbsoluteEncoder.get())
                    * (isWristExternalEncoderInverted ? -1 : 1)
                - wristAbsoluteEncoderOffset.getRadians());
    inputs.wristRelativePositionRad =
        wristRelativeEncoder.getDistance() * (isWristExternalEncoderInverted ? -1 : 1);
    inputs.wristInternalPositionRad =
        Units.rotationsToRadians(wristInternalEncoder.getPosition())
            / config.wrist().motor().reduction();
    inputs.wristRelativeVelocityRadPerSec =
        wristRelativeEncoder.getRate() * (isWristExternalEncoderInverted ? -1 : 1);
    inputs.wristInternalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristInternalEncoder.getPosition())
            / config.wrist().motor().reduction();
    inputs.wristAppliedVolts = wristSparkMax.getAppliedOutput() * wristSparkMax.getBusVoltage();
    inputs.wristCurrentAmps = new double[] {wristSparkMax.getOutputCurrent()};
    inputs.wristTempCelcius = new double[] {wristSparkMax.getMotorTemperature()};
  }

  public void setShoulderVoltage(double volts) {
    shoulderSparkMax.setVoltage(volts);
  }

  public void setElbowVoltage(double volts) {
    elbowSparkMax.setVoltage(volts);
  }

  public void setWristVoltage(double volts) {
    wristSparkMax.setVoltage(volts);
  }

  public void setBrakeMode(boolean shoulderBrake, boolean elbowBrake, boolean wristBrake) {
    shoulderSparkMax.setIdleMode(shoulderBrake ? IdleMode.kBrake : IdleMode.kCoast);
    shoulderSparkMaxFollower.setIdleMode(shoulderBrake ? IdleMode.kBrake : IdleMode.kCoast);
    elbowSparkMax.setIdleMode(elbowBrake ? IdleMode.kBrake : IdleMode.kCoast);
    wristSparkMax.setIdleMode(wristBrake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
