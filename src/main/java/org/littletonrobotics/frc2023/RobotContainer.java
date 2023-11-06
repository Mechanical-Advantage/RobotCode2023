// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.commands.DriveWithJoysticks;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import org.littletonrobotics.frc2023.commands.autos.*;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {

  // Subsystems
  private Drive drive;
  private CubeIntake cubeIntake;

  // OI objects
  private final CommandJoystick driverLeft = new CommandJoystick(0);
  private final CommandJoystick driverRight = new CommandJoystick(1);
  private final CommandXboxController operator = new CommandXboxController(2);

  // Alerts
  private final Alert driverLeftDisconnected =
      new Alert("Left driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert driverRightDisconnected =
      new Alert("Right driver controller disconnected (port 1).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 2).", AlertType.WARNING);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
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

    // Set up auto routines
    autoChooser.addDefaultOption(
        "Do Almost Nothing",
        Commands.runOnce(
            () ->
                drive.setPose(
                    AllianceFlipUtil.apply(
                        new Pose2d(new Translation2d(), new Rotation2d(Math.PI))))));
    autoChooser.addOption("Score and Do Nothing", new ScoreAndDoNothing(drive, cubeIntake));
    autoChooser.addOption("Wallside Two Piece", new WallsideTwoPiece(drive, cubeIntake, false));
    autoChooser.addOption(
        "Wallside Two Piece Balance", new WallsideTwoPiece(drive, cubeIntake, true));
    autoChooser.addOption("Wallside Three Piece", new WallsideThreePiece(drive, cubeIntake));
    autoChooser.addOption("Fieldside Two Piece", new FieldsideTwoPiece(drive, cubeIntake, false));
    autoChooser.addOption(
        "Fieldside Two Piece Balance", new FieldsideTwoPiece(drive, cubeIntake, true));
    autoChooser.addOption("Fieldside Three Piece", new FieldSideThreePiece(drive, cubeIntake));
    autoChooser.addOption("Score and Balance", new ScoreAndBalance(drive, cubeIntake));
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drive,
            true,
            new FeedForwardCharacterizationData("drive"),
            (Double voltage) -> drive.runCharacterizationVolts(voltage),
            drive::getCharacterizationVelocity));

    autoChooser.addOption("New Path", new TestPathAuto(drive));

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
                      operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                    })
                .withTimeout(1.5)
                .andThen(
                    Commands.run(
                            () -> {
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
                          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
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
    driverLeftDisconnected.set(
        !DriverStation.isJoystickConnected(driverLeft.getHID().getPort())
            || DriverStation.getJoystickIsXbox(driverLeft.getHID().getPort()));
    driverRightDisconnected.set(
        !DriverStation.isJoystickConnected(driverRight.getHID().getPort())
            || DriverStation.getJoystickIsXbox(driverRight.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
  }

  /** Binds the driver and operator controls. */
  public void bindControls() {
    // Clear old buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // *** DRIVER CONTROLS ***
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> -driverLeft.getRawAxis(1),
            () -> -driverLeft.getRawAxis(0),
            () -> -driverRight.getRawAxis(0)));
    driverRight
        .button(1)
        .or(driverRight.button(2))
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              AllianceFlipUtil.apply(new Rotation2d())));
                    })
                .ignoringDisable(true));
    driverLeft.button(1).or(driverLeft.button(2)).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // *** OPERATOR CONTROLS ***
    operator.b().whileTrue(cubeIntake.ejectCommand());
    operator.a().whileTrue(cubeIntake.intakeCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
