// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

  private final MechanismLigament2d armElevator;
  private final MechanismLigament2d armWrist;
  private final double kElevatorLength = 0.5;

  private Mechanism2d mech = new Mechanism2d(3, 3);
  private MechanismRoot2d root = mech.getRoot("arm", 2, 0);
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

    armElevator = root.append(new MechanismLigament2d("elevator", kElevatorLength, 90));
    armWrist = armElevator.append(new MechanismLigament2d("wrist", kElevatorLength, 90));
  }

  public void periodic() {
    io.updateInputs(armInputs);
    Logger.getInstance().processInputs("Arm", armInputs);

    armWrist.setAngle(Units.radiansToDegrees(armInputs.positionRad) - 90);

    Logger.getInstance().recordOutput("Arm/Length", mech);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }
}
