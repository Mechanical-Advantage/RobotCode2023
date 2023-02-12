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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class ArmIOSparkMax implements ArmIO {
  private ArmConfig config;

  private final CANSparkMax shoulderSparkMax;
  private final CANSparkMax shoulderSparkMaxFollower;
  private final PWM shoulderAbsoluteEncoder;
  private final Encoder shoulderRelativeEncoder;
  private final RelativeEncoder shoulderInternalEncoder;
  private final boolean isShoulderMotorInverted;
  private final boolean isShoulderExternalEncoderInverted;
  private final Rotation2d shoulderAbsoluteEncoderOffset;

  private final CANSparkMax elbowSparkMax;
  private final PWM elbowAbsoluteEncoder;
  private final Encoder elbowRelativeEncoder;
  private final RelativeEncoder elbowInternalEncoder;
  private final boolean isElbowMotorInverted;
  private final boolean isElbowExternalEncoderInverted;
  private final Rotation2d elbowAbsoluteEncoderOffset;

  private final CANSparkMax wristSparkMax;
  private final PWM wristAbsoluteEncoder;
  private final Encoder wristRelativeEncoder;
  private final RelativeEncoder wristInternalEncoder;
  private final boolean isWristMotorInverted;
  private final boolean isWristExternalEncoderInverted;
  private final Rotation2d wristAbsoluteEncoderOffset;

  public ArmIOSparkMax() {
    // Shoulder config
    shoulderSparkMax = new CANSparkMax(15, MotorType.kBrushless);
    shoulderSparkMaxFollower = new CANSparkMax(16, MotorType.kBrushless);
    shoulderAbsoluteEncoder = new PWM(0, false);
    shoulderRelativeEncoder = new Encoder(0, 1, false);
    isShoulderMotorInverted = false;
    isShoulderExternalEncoderInverted = false;
    shoulderAbsoluteEncoderOffset = new Rotation2d(0.0);
    shoulderRelativeEncoder.setDistancePerPulse((2 * Math.PI) / 8192);

    // Elbow config
    elbowSparkMax = new CANSparkMax(17, MotorType.kBrushless);
    elbowAbsoluteEncoder = new PWM(0, false);
    elbowRelativeEncoder = new Encoder(0, 1, false);
    isElbowMotorInverted = false;
    isElbowExternalEncoderInverted = false;
    elbowAbsoluteEncoderOffset = new Rotation2d(0.0);
    elbowRelativeEncoder.setDistancePerPulse((2 * Math.PI) / 8192);

    // Wrist config
    wristSparkMax = new CANSparkMax(18, MotorType.kBrushless);
    wristAbsoluteEncoder = new PWM(0, false);
    wristRelativeEncoder = new Encoder(0, 1, false);
    isWristMotorInverted = false;
    isWristExternalEncoderInverted = false;
    wristAbsoluteEncoderOffset = new Rotation2d(0.0);
    wristRelativeEncoder.setDistancePerPulse((2 * Math.PI) / 8192);

    // Set remaining config
    if (SparkMaxBurnManager.shouldBurn()) {
      shoulderSparkMax.restoreFactoryDefaults();
      shoulderSparkMaxFollower.restoreFactoryDefaults();
      elbowSparkMax.restoreFactoryDefaults();
      wristSparkMax.restoreFactoryDefaults();
    }

    shoulderSparkMaxFollower.follow(shoulderSparkMax);
    shoulderSparkMax.setInverted(isShoulderMotorInverted);
    elbowSparkMax.setInverted(isElbowMotorInverted);
    wristSparkMax.setInverted(isWristMotorInverted);

    shoulderSparkMax.setSmartCurrentLimit(30);
    shoulderSparkMaxFollower.setSmartCurrentLimit(30);
    elbowSparkMax.setSmartCurrentLimit(30);
    wristSparkMax.setSmartCurrentLimit(30);

    shoulderSparkMax.enableVoltageCompensation(12.0);
    elbowSparkMax.enableVoltageCompensation(12.0);
    wristSparkMax.enableVoltageCompensation(12.0);

    shoulderInternalEncoder = shoulderSparkMax.getEncoder();
    shoulderInternalEncoder.setMeasurementPeriod(10);
    shoulderInternalEncoder.setAverageDepth(2);
    shoulderInternalEncoder.setPosition(0.0);

    elbowInternalEncoder = elbowSparkMax.getEncoder();
    elbowInternalEncoder.setMeasurementPeriod(10);
    elbowInternalEncoder.setAverageDepth(2);
    elbowInternalEncoder.setPosition(0.0);

    wristInternalEncoder = wristSparkMax.getEncoder();
    wristInternalEncoder.setMeasurementPeriod(10);
    wristInternalEncoder.setAverageDepth(2);
    wristInternalEncoder.setPosition(0.0);

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
            shoulderAbsoluteEncoder.getPosition()
                    * 2
                    * Math.PI
                    * (isShoulderExternalEncoderInverted ? -1 : 1)
                - shoulderAbsoluteEncoderOffset.getRadians());
    inputs.shoulderRelativePositionRad =
        shoulderRelativeEncoder.getDistance() * (isShoulderExternalEncoderInverted ? -1 : 1);
    inputs.shoulderInternalPositionRad =
        Units.rotationsToRadians(shoulderInternalEncoder.getPosition())
            / config.shoulder().reduction();
    inputs.shoulderRelativeVelocityRadPerSec =
        shoulderRelativeEncoder.getRate() * (isShoulderExternalEncoderInverted ? -1 : 1);
    inputs.shoulderInternalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shoulderInternalEncoder.getPosition())
            / config.shoulder().reduction();
    inputs.shoulderAppliedVolts =
        shoulderSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.shoulderCurrentAmps = new double[] {shoulderSparkMax.getOutputCurrent()};
    inputs.shoulderTempCelcius = new double[] {shoulderSparkMax.getMotorTemperature()};

    inputs.shoulderAbsolutePositionRad =
        MathUtil.angleModulus(
            elbowAbsoluteEncoder.getPosition()
                    * 2
                    * Math.PI
                    * (isElbowExternalEncoderInverted ? -1 : 1)
                - elbowAbsoluteEncoderOffset.getRadians());
    inputs.elbowRelativePositionRad =
        elbowRelativeEncoder.getDistance() * (isElbowExternalEncoderInverted ? -1 : 1);
    inputs.elbowInternalPositionRad =
        Units.rotationsToRadians(elbowInternalEncoder.getPosition()) / config.elbow().reduction();
    inputs.elbowRelativeVelocityRadPerSec =
        elbowRelativeEncoder.getRate() * (isElbowExternalEncoderInverted ? -1 : 1);
    inputs.elbowInternalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(elbowInternalEncoder.getPosition())
            / config.elbow().reduction();
    inputs.elbowAppliedVolts =
        elbowSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.elbowCurrentAmps = new double[] {elbowSparkMax.getOutputCurrent()};
    inputs.elbowTempCelcius = new double[] {elbowSparkMax.getMotorTemperature()};

    inputs.wristAbsolutePositionRad =
        MathUtil.angleModulus(
            wristAbsoluteEncoder.getPosition()
                    * 2
                    * Math.PI
                    * (isWristExternalEncoderInverted ? -1 : 1)
                - wristAbsoluteEncoderOffset.getRadians());
    inputs.wristRelativePositionRad =
        wristRelativeEncoder.getDistance() * (isWristExternalEncoderInverted ? -1 : 1);
    inputs.wristInternalPositionRad =
        Units.rotationsToRadians(wristInternalEncoder.getPosition()) / config.wrist().reduction();
    inputs.wristRelativeVelocityRadPerSec =
        wristRelativeEncoder.getRate() * (isWristExternalEncoderInverted ? -1 : 1);
    inputs.wristInternalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wristInternalEncoder.getPosition())
            / config.wrist().reduction();
    inputs.wristAppliedVolts =
        wristSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
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
    elbowSparkMax.setIdleMode(elbowBrake ? IdleMode.kBrake : IdleMode.kCoast);
    wristSparkMax.setIdleMode(wristBrake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
