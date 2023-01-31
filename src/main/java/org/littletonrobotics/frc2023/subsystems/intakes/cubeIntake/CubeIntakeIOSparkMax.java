// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intakes.cubeIntake;

import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.SparkMaxDerivedVelocityController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class CubeIntakeIOSparkMax implements CubeIntakeIO 
{
  private boolean armInvert = false;
  private boolean intakeInvert = false;

  private double armAfterEncoderReduction = 1.0;
  private double intakeAfterEncoderReduction = 1.0;

  private CANSparkMax armSparkMax;
  private CANSparkMax intakeSparkMax;

  private SparkMaxDerivedVelocityController armSparkMaxDerivedVelocityController;
  private SparkMaxDerivedVelocityController intakeSparkMaxDerivedVelocityController;

  private RelativeEncoder armRelativeEncoder;
  private RelativeEncoder intakeRelativeEncoder;

  private SparkMaxAbsoluteEncoder armAbsoluteEncoder;
  private SparkMaxAbsoluteEncoder intakeAbsoluteEncoder;

  public CubeIntakeIOSparkMax() 
  {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        armSparkMax = new CANSparkMax(0, MotorType.kBrushless);
        intakeSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        armAbsoluteEncoder = armSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        intakeAbsoluteEncoder = intakeSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        break;
      default:
        throw new RuntimeException("Invalid robot for IntakeIOSparkMax!");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.restoreFactoryDefaults();
      intakeSparkMax.restoreFactoryDefaults();
    }

    armSparkMax.setInverted(armInvert);
    intakeSparkMax.setInverted(intakeInvert);

    armSparkMax.setSmartCurrentLimit(30);
    intakeSparkMax.setSmartCurrentLimit(30);

    armSparkMax.enableVoltageCompensation(12.0);
    intakeSparkMax.enableVoltageCompensation(12.0);

    armRelativeEncoder = armSparkMax.getEncoder();
    intakeRelativeEncoder = intakeSparkMax.getEncoder();

    armRelativeEncoder.setPosition(0.0);
    intakeRelativeEncoder.setPosition(0.0);

    armSparkMax.setCANTimeout(0);
    intakeSparkMax.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
      intakeSparkMax.burnFlash();
    }
  }
  
  @Override
  public void updateInputs(CubeIntakeIOInputs inputs)
  {
    // update arm motor inputs
    inputs.armPositionRad = Units.rotationsToRadians(armRelativeEncoder.getPosition()) / armAfterEncoderReduction;
    inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(armRelativeEncoder.getVelocity())
        / armAfterEncoderReduction;
    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.armCurrentAmps = new double[] { armSparkMax.getOutputCurrent() };
    inputs.armTempCelcius = new double[] { armSparkMax.getMotorTemperature() };

    // update intake motor inputs
  }

  @Override
  public void setArmVoltage(double voltage)
  {
    armSparkMax.setVoltage(voltage);
  }

  @Override
  public void setIntakeVoltage(double voltage)
  {
    intakeSparkMax.setVoltage(voltage);
  }
}
