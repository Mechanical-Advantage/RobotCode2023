// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIO.ArmIOInputs;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        break;
      case ROBOT_SIMBOT:
        break;
      default:
        break;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void periodic()
  {
    io.updateInputs(armInputs);
    Logger.getInstance().processInputs("Arm", armInputs);
  }

  public void updateInputs(ArmIOInputs inputs)
  {
    io.updateInputs(inputs);
  }

  public void setVoltage(double volts)
  {
    io.setVoltage(volts);
  }
}
