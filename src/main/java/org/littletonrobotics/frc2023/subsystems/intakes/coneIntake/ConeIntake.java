// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intakes.coneIntake;

import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConeIntake extends SubsystemBase {
  /** Creates a new ConeIntake. */
  private ConeIntakeIO coneIO;
  private final ConeIntakeIOInputsAutoLogged ConeIntakeInputs = new ConeIntakeIOInputsAutoLogged();

  private MechanismLigament2d armElevator;
  private MechanismLigament2d armWrist;

  private double armMechLength = 3.0;
  private double armMechWidth = 3.0;
  private double kElevatorLength = 0.5;

  private Mechanism2d armMech = new Mechanism2d(armMechLength, armMechWidth);
  private MechanismRoot2d armMechRoot = armMech.getRoot("arm", 2, 0);

  /** Creates a new Arm. */
  public void Arm(ConeIntakeIO coneIO) {
    this.coneIO = coneIO;
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        break;
      case ROBOT_2023P:
        break;
      case ROBOT_SIMBOT:
        break;
      default:
        break;
    }

    armElevator = armMechRoot.append(new MechanismLigament2d("elevator", kElevatorLength, 90));
    armWrist = armElevator.append(new MechanismLigament2d("wrist", kElevatorLength, armMechLength));
  }

  @Override
  public void periodic() {
    coneIO.updateInputs(ConeIntakeInputs);
    Logger.getInstance().processInputs("ConeIntake", ConeIntakeInputs);

    armWrist.setAngle(Units.radiansToDegrees(ConeIntakeInputs.armPositionRad) - 90);

    Logger.getInstance().recordOutput("ConeIntake/Length", armMech);
  }

  public void setIntakeVoltage(double volts)
  {
    coneIO.setIntakeVoltage(volts);
  }

  public void setArmVoltage(double volts)
  {
    coneIO.setArmVoltage(volts);
  }
}