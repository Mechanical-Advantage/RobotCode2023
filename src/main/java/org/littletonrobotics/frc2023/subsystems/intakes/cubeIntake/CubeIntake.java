// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.intakes.cubeIntake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.junction.Logger;

public class CubeIntake extends SubsystemBase {
  /** Creates a new CubeIntake. */
  private CubeIntakeIO cubeIO;

  private final CubeIntakeIOInputsAutoLogged cubeIntakeInputs = new CubeIntakeIOInputsAutoLogged();

  private MechanismLigament2d armElevator;
  private MechanismLigament2d armWrist;

  private double armMechLength = 3.0;
  private double armMechWidth = 3.0;
  private double kElevatorLength = 0.5;

  private Mechanism2d armMech = new Mechanism2d(armMechLength, armMechWidth);
  private MechanismRoot2d armMechRoot = armMech.getRoot("arm", 2, 0);
  
  private double armP = 1.0;
  private double armI = 1.0;
  private double armD = 1.0;

  private PIDController armPositionPidController = new PIDController(armP, armI, armD);

  /** Creates a new Arm. */
  public CubeIntake(CubeIntakeIO cubeIO) {
    this.cubeIO = cubeIO;
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
    cubeIO.updateInputs(cubeIntakeInputs);
    Logger.getInstance().processInputs("CubeIntake", cubeIntakeInputs);

    armWrist.setAngle(Units.radiansToDegrees(cubeIntakeInputs.armPositionRad) - 90);

    Logger.getInstance().recordOutput("CubeIntake/Length", armMech);
  }

  public void runIntakePercent(double percent) {
    cubeIO.setIntakeVoltage(percent * 12.0);
  }

  public void setArmPosition(double positionRads) {
    cubeIO.setArmVoltage(
        armPositionPidController.calculate(cubeIntakeInputs.armAbsolutePosition, positionRads));
  }
}
