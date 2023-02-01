// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intakes.cubeIntake;

import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
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

  private RelativeEncoder armRelativeEncoder;
  private RelativeEncoder intakeRelativeEncoder;

  private SparkMaxAbsoluteEncoder armAbsoluteEncoder;
  private Rotation2d armAbsoluteEncoderOffset;

  public CubeIntakeIOSparkMax() 
  {
    switch (Constants.getRobot()) 
    {
      case ROBOT_2023C:
        armSparkMax = new CANSparkMax(0, MotorType.kBrushless);
        intakeSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        armAbsoluteEncoderOffset = new Rotation2d(0.0);
        break;
        default:
        throw new RuntimeException("Invalid robot for IntakeIOSparkMax!");
      }
      
      if (SparkMaxBurnManager.shouldBurn()) 
      {
        armSparkMax.restoreFactoryDefaults();
        intakeSparkMax.restoreFactoryDefaults();
      }
      
    armAbsoluteEncoder = armSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    intakeRelativeEncoder.setPosition(0.0);
    armRelativeEncoder.setPosition(armAbsoluteEncoderOffset.getRotations());
    
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
    inputs.armAbsolutePosition = Units.rotationsToRadians(armAbsoluteEncoder.getPosition());
    inputs.armPositionRad = Units.rotationsToRadians((new Rotation2d(armAbsoluteEncoder.getPosition()).minus(armAbsoluteEncoderOffset)).getRotations() / armAfterEncoderReduction);
    inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(armRelativeEncoder.getVelocity())
        / armAfterEncoderReduction;
    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.armCurrentAmps = new double[] { armSparkMax.getOutputCurrent() };
    inputs.armTempCelcius = new double[] { armSparkMax.getMotorTemperature() };

    // update intake motor inputs
    inputs.intakePositionRad = Units
        .rotationsToRadians((new Rotation2d(intakeRelativeEncoder.getPosition()).minus(armAbsoluteEncoderOffset)).getRotations() / intakeAfterEncoderReduction);
    inputs.intakeAppliedVolts = intakeSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.intakeCurrentAmps = new double[] { intakeSparkMax.getOutputCurrent() };
    inputs.intakeTempCelcius = new double[] { intakeSparkMax.getMotorTemperature() };
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
