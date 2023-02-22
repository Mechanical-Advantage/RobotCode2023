// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.gripper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.frc2023.util.SuppliedWaitCommand;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  private static final LoggedTunableNumber holdVolts =
      new LoggedTunableNumber("Gripper/HoldVolts", 0.0);
  private static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Gripper/IntakeVolts", 10.0);
  private static final LoggedTunableNumber ejectVolts =
      new LoggedTunableNumber("Gripper/EjectVolts", -10.0);
  private static final LoggedTunableNumber ejectSecs =
      new LoggedTunableNumber("Gripper/EjectSecs", 0.25);

  public Gripper(GripperIO io) {
    this.io = io;
    io.setBrakeMode(false);
    setDefaultCommand(
        run(
            () -> {
              io.setVoltage(holdVolts.get());
            }));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Gripper", inputs);

    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
    }
  }

  /** Command factory to run the gripper wheels forward and grab a game piece. */
  public Command intakeCommand() {
    return run(() -> io.setVoltage(intakeVolts.get()));
  }

  /** Command factory to run the gripper wheels back and eject a game piece. */
  public Command ejectCommand() {
    return run(() -> io.setVoltage(ejectVolts.get()))
        .raceWith(new SuppliedWaitCommand(() -> ejectSecs.get()));
  }
}
