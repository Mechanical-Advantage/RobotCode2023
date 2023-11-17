// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.commands.DriveWithJoysticks;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntake;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntakeIO;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntakeIOSim;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntakeIOSparkMax;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.drive.GyroIO;
import org.littletonrobotics.frc2023.subsystems.drive.GyroIOPigeon2;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIO;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {

  // Subsystems
  private Drive drive;

  private CubeIntake cubeIntake;

  // OI objects
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 15.0);

  public RobotContainer() {
    // Check if flash should be burned
    SparkMaxBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));

          break;
        case ROBOT_2023P:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));

          cubeIntake = new CubeIntake(new CubeIntakeIOSparkMax());

          break;
        case ROBOT_SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());

          cubeIntake = new CubeIntake(new CubeIntakeIOSim());

          break;
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    if (cubeIntake == null) {
      cubeIntake = new CubeIntake(new CubeIntakeIO() {});
    }

    // Set up subsystems

    // Set up auto routines

    // System.out.println("[Init] Instantiating auto routines (Drive Characterization)");
    // autoSelector.addRoutine(
    //     "Drive Characterization",
    //     List.of(),
    //     new FeedForwardCharacterization(
    //         drive,
    //         true,
    //         new FeedForwardCharacterizationData("drive"),
    //         drive::runCharacterizationVolts,
    //         drive::getCharacterizationVelocity));

    // Startup alerts
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }
    if (FieldConstants.isWPIField) {
      new Alert("WPI field selected, do not use in competition.", AlertType.INFO).set(true);
    }

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.run(
                    () -> {
                      driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                      operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                    })
                .withTimeout(1.5)
                .andThen(
                    Commands.run(
                            () -> {
                              driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                              operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                            })
                        .withTimeout(1.0)));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                Commands.run(
                        () -> {
                          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(1.0)));

    // Bind driver and operator controls
    System.out.println("[Init] Binding controls");
    bindControls();

    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
  }

  /** Binds the driver and operator controls. */
  public void bindControls() {
    // Clear old buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();


    // Drive controls

    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    driver
        .start()
        .or(driver.back())
        .and(() -> DriverStation.isDisabled())
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              AllianceFlipUtil.apply(new Rotation2d())));
                    })
                .ignoringDisable(true));

    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    operator.b().whileTrue(cubeIntake.ejectCommand());
    operator.a().whileTrue(cubeIntake.intakeCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
