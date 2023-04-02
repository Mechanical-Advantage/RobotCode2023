// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.gripper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.subsystems.leds.Leds;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.frc2023.util.SuppliedWaitCommand;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  private static final LoggedTunableNumber holdVolts =
      new LoggedTunableNumber("Gripper/HoldVolts", 1.2);
  private static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Gripper/IntakeVolts", 12.0);
  private static final LoggedTunableNumber intakeVelocityWaitStart =
      new LoggedTunableNumber("Gripper/IntakeVelocityWaitStart", 0.3);
  private static final LoggedTunableNumber intakeVelocityWaitStop =
      new LoggedTunableNumber("Gripper/IntakeVelocityWaitStop", 0.5);
  private static final LoggedTunableNumber intakeStopVelocity =
      new LoggedTunableNumber("Gripper/IntakeStopVelocity", 0.4);
  private static final LoggedTunableNumber ejectVoltsVeryFast =
      new LoggedTunableNumber("Gripper/EjectVoltsVeryFast", -12.0);
  private static final LoggedTunableNumber ejectVoltsFast =
      new LoggedTunableNumber("Gripper/EjectVoltsFast", -3.5);
  private static final LoggedTunableNumber ejectSecsFast =
      new LoggedTunableNumber("Gripper/EjectSecsFast", 0.4);
  private static final LoggedTunableNumber ejectVoltsMedium =
      new LoggedTunableNumber("Gripper/EjectVoltsMedium", -3.0);
  private static final LoggedTunableNumber ejectVoltsSlow =
      new LoggedTunableNumber("Gripper/EjectVoltsSlow", -2.0);
  private static final LoggedTunableNumber ejectSecsSlow =
      new LoggedTunableNumber("Gripper/EjectSecsSlow", 0.6);

  private static final double tooHotTemperatureHigh = 80.0;
  private static final double tooHotTemperatureHighTime = 2.0;
  private static final double tooHotTemperatureLow = 65.0;
  private boolean tooHotAlertActive = false;
  private final Timer tooHotTimer = new Timer();
  private final Alert tooHotAlert =
      new Alert("Gripper motor disabled due to very high temperature.", AlertType.ERROR);
  private Timer dsEnabledTimer = new Timer();
  private final Debouncer stoppedLedsDebouncer = new Debouncer(0.1, DebounceType.kBoth);
  private Supplier<Boolean> forceEnableSupplier = () -> false;

  public Gripper(GripperIO io) {
    this.io = io;
    io.setBrakeMode(false);
    tooHotTimer.start();
    dsEnabledTimer.start();
    setDefaultCommand(
        run(() -> {
              setVoltage(holdVolts.get());
            })
            .withName("GripperIdle"));
  }

  public void setOverrides(Supplier<Boolean> forceEnableSupplier) {
    this.forceEnableSupplier = forceEnableSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Gripper", inputs);

    // Stop movement when disabled
    if (DriverStation.isDisabled() || tooHotAlertActive) {
      io.setVoltage(0.0);
    }

    // Update gripper stopped LEDs
    if (DriverStation.isDisabled()) {
      dsEnabledTimer.reset();
    }
    Leds.getInstance().gripperStopped =
        stoppedLedsDebouncer.calculate(
                Math.abs(inputs.velocityRadPerSec) < intakeStopVelocity.get())
            && dsEnabledTimer.hasElapsed(intakeVelocityWaitStart.get());

    // Update too hot alert
    if (inputs.tempCelcius.length == 0 || inputs.tempCelcius[0] < tooHotTemperatureHigh) {
      tooHotTimer.reset();
    }
    if (tooHotAlertActive) {
      if (inputs.tempCelcius[0] < tooHotTemperatureLow) {
        tooHotAlertActive = false;
      }
    } else {
      if (tooHotTimer.hasElapsed(tooHotTemperatureHighTime)) {
        tooHotAlertActive = true;
      }
    }
    if (inputs.tempCelcius.length > 0 && tooHotAlertActive) {
      tooHotAlert.setText(
          "Gripper motor disabled due to very high temperature ("
              + Long.toString(Math.round(inputs.tempCelcius[0]))
              + "°C)");
    }
    tooHotAlert.set(tooHotAlertActive);
  }

  private void setVoltage(double volts) {
    if (!tooHotAlertActive || forceEnableSupplier.get()) {
      io.setVoltage(volts);
    }
  }

  /** Command factory to run the gripper wheels forward and grab a game piece. */
  public Command intakeCommand() {
    return run(() -> setVoltage(intakeVolts.get()))
        .raceWith(
            new SuppliedWaitCommand(() -> intakeVelocityWaitStart.get())
                .andThen(
                    Commands.waitUntil(() -> inputs.velocityRadPerSec < intakeStopVelocity.get()),
                    new SuppliedWaitCommand(() -> intakeVelocityWaitStop.get())))
        .finallyDo((interrupted) -> setVoltage(holdVolts.get()));
  }

  /** Command factory to run the gripper wheels back and eject a game piece. */
  public Command ejectCommand(Objective objective) {
    return Commands.either(
        ejectCommand(EjectSpeed.SLOW),
        Commands.either(
            ejectCommand(EjectSpeed.MEDIUM),
            ejectCommand(EjectSpeed.FAST),
            () -> objective.nodeLevel == NodeLevel.HYBRID),
        () -> objective.isConeNode());
  }

  /** Command factory to run the gripper wheels back and eject a game piece. */
  public Command ejectCommand(EjectSpeed speed) {
    Supplier<Double> voltsSupplier;
    switch (speed) {
      case VERY_FAST:
        voltsSupplier = () -> ejectVoltsVeryFast.get();
        break;
      case FAST:
        voltsSupplier = () -> ejectVoltsFast.get();
        break;
      case MEDIUM:
        voltsSupplier = () -> ejectVoltsMedium.get();
        break;
      case SLOW:
        voltsSupplier = () -> ejectVoltsSlow.get();
        break;
      default:
        voltsSupplier = () -> 0.0;
        break;
    }
    return run(() -> setVoltage(voltsSupplier.get()))
        .raceWith(
            new SuppliedWaitCommand(
                () ->
                    speed == EjectSpeed.SLOW || speed == EjectSpeed.MEDIUM
                        ? ejectSecsSlow.get()
                        : ejectSecsFast.get()))
        .finallyDo((interrupted) -> setVoltage(holdVolts.get()));
  }

  public static enum EjectSpeed {
    VERY_FAST,
    FAST,
    MEDIUM,
    SLOW
  }
}
