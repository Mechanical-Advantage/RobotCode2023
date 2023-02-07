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
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class GripperIOSparkMax implements GripperIO {
  private boolean invert = false;
  private final CANSparkMax motor;

  public GripperIOSparkMax() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        motor = new CANSparkMax(4, MotorType.kBrushed);
        invert = false;
        break;
      default:
        throw new RuntimeException("Invalid robot for GripperIOSparkMax!");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      motor.restoreFactoryDefaults();
    }

    motor.setInverted(invert);
    motor.setSmartCurrentLimit(10);
    motor.enableVoltageCompensation(12.0);

    motor.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      motor.burnFlash();
    }
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
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
