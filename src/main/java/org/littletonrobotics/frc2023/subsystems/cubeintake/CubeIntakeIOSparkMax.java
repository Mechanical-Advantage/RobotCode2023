// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.cubeintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

/** Add your docs here. */
public class CubeIntakeIOSparkMax implements CubeIntakeIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax rollerSparkMax;

  private final PWM armAbsoluteEncoder;
  private final Encoder armRelativeEncoder;
  private final RelativeEncoder armInternalEncoder;

  private final boolean armInvert;
  private final boolean armExternalEncoderInvert;
  private final boolean rollerInvert;
  private final double armInternalEncoderReduction;
  private final Rotation2d armAbsoluteEncoderOffset;

  public CubeIntakeIOSparkMax() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        armSparkMax = new CANSparkMax(0, MotorType.kBrushless);
        rollerSparkMax = new CANSparkMax(1, MotorType.kBrushless);

        armInvert = false;
        armExternalEncoderInvert = false;
        rollerInvert = false;
        armInternalEncoderReduction = 1.0;
        armAbsoluteEncoderOffset = new Rotation2d(0.0);

        armAbsoluteEncoder = new PWM(0, false);
        armRelativeEncoder = new Encoder(0, 1, false);
        armRelativeEncoder.setDistancePerPulse((2 * Math.PI) / 8192);
        break;
      default:
        throw new RuntimeException("Invalid robot for CubeIntakeIOSparkMax!");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.restoreFactoryDefaults();
      rollerSparkMax.restoreFactoryDefaults();
    }

    armInternalEncoder = armSparkMax.getEncoder();
    armInternalEncoder.setPosition(0.0);
    armInternalEncoder.setMeasurementPeriod(10);
    armInternalEncoder.setAverageDepth(2);

    armSparkMax.setInverted(armInvert);
    rollerSparkMax.setInverted(rollerInvert);

    armSparkMax.setSmartCurrentLimit(30);
    rollerSparkMax.setSmartCurrentLimit(30);

    armSparkMax.enableVoltageCompensation(12.0);
    rollerSparkMax.enableVoltageCompensation(12.0);

    armSparkMax.setCANTimeout(0);
    rollerSparkMax.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
      rollerSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(CubeIntakeIOInputs inputs) {
    inputs.armAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    armAbsoluteEncoder.getPosition()
                        * 2
                        * Math.PI
                        * (armExternalEncoderInvert ? -1 : 1))
                .minus(armAbsoluteEncoderOffset)
                .getRadians());
    inputs.armRelativePositionRad =
        armRelativeEncoder.getDistance() * (armExternalEncoderInvert ? -1 : 1);
    inputs.armInternalPositionRad =
        Units.rotationsToRadians(armInternalEncoder.getPosition()) / armInternalEncoderReduction;
    inputs.armRelativeVelocityRadPerSec =
        armRelativeEncoder.getRate() * (armExternalEncoderInvert ? -1 : 1);
    inputs.armInternalVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armInternalEncoder.getPosition())
            / armInternalEncoderReduction;
    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.armCurrentAmps = new double[] {armSparkMax.getOutputCurrent()};
    inputs.armTempCelcius = new double[] {armSparkMax.getMotorTemperature()};

    inputs.rollerAppliedVolts =
        rollerSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.rollerCurrentAmps = new double[] {rollerSparkMax.getOutputCurrent()};
  }

  @Override
  public void setArmVoltage(double voltage) {
    armSparkMax.setVoltage(voltage);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerSparkMax.setVoltage(voltage);
  }
}
