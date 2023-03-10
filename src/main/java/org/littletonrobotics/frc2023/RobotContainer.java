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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.AutoSelector.AutoQuestion;
import org.littletonrobotics.frc2023.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.commands.AutoCommands;
import org.littletonrobotics.frc2023.commands.AutoScore;
import org.littletonrobotics.frc2023.commands.DriveToSubstation;
import org.littletonrobotics.frc2023.commands.DriveWithJoysticks;
import org.littletonrobotics.frc2023.commands.EjectHeld;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import org.littletonrobotics.frc2023.commands.IntakeConeFloor;
import org.littletonrobotics.frc2023.commands.IntakeCubeHandoff;
import org.littletonrobotics.frc2023.commands.IntakeSubstation;
import org.littletonrobotics.frc2023.commands.MoveArmWithJoysticks;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIONorthstar;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIO;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIOSim;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIOSparkMax;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.arm.ArmSolverIO;
import org.littletonrobotics.frc2023.subsystems.arm.ArmSolverIOKairos;
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
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper.EjectSpeed;
import org.littletonrobotics.frc2023.subsystems.gripper.GripperIO;
import org.littletonrobotics.frc2023.subsystems.gripper.GripperIOSparkMax;
import org.littletonrobotics.frc2023.subsystems.leds.Leds;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIOServer;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Direction;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.DoublePressTracker;
import org.littletonrobotics.frc2023.util.OverrideSwitches;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class RobotContainer {

  // Subsystems
  private Drive drive;
  private Arm arm;
  private Gripper gripper;
  private CubeIntake cubeIntake;
  private AprilTagVision aprilTagVision;
  private ObjectiveTracker objectiveTracker;

  // OI objects
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger robotRelative = overrides.driverSwitch(0);
  private final Trigger armDisable = overrides.driverSwitch(1);
  private final Trigger armCoast = overrides.driverSwitch(2);
  private final Trigger hpDoubleSubstationSwitch = overrides.multiDirectionSwitchLeft();
  private final Trigger hpThrowGamePieceSwitch = overrides.multiDirectionSwitchRight();
  private final Trigger manualDrive = overrides.operatorSwitch(0);
  private final Trigger autoEject = overrides.operatorSwitch(1);
  private final Trigger reachScoringEnable = overrides.operatorSwitch(2);
  private final Trigger forcePregenPaths = overrides.operatorSwitch(3);
  private final Trigger forceGripperEnable = overrides.operatorSwitch(4);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.INFO);
  private final LoggedDashboardNumber endgameAlertTime =
      new LoggedDashboardNumber("Endgame Alert Time", 30.0);

  // Auto selector
  private final AutoSelector autoSelector = new AutoSelector("Auto");

  public RobotContainer() {
    // Check if flash should be burned
    SparkMaxBurnManager.update();

    // Instantiate LEDs
    Leds.getInstance();

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
          arm = new Arm(new ArmIOSparkMax(), new ArmSolverIOKairos(4));
          gripper = new Gripper(new GripperIOSparkMax());
          cubeIntake = new CubeIntake(new CubeIntakeIOSparkMax());
          aprilTagVision =
              new AprilTagVision(
                  new AprilTagVisionIONorthstar("northstar_0"),
                  new AprilTagVisionIONorthstar("northstar_1"),
                  new AprilTagVisionIONorthstar("northstar_2"),
                  new AprilTagVisionIONorthstar("northstar_3"));
          objectiveTracker = new ObjectiveTracker(new NodeSelectorIOServer());
          break;
        case ROBOT_2023P:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));
          objectiveTracker = new ObjectiveTracker(new NodeSelectorIOServer());
          break;
        case ROBOT_SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          arm = new Arm(new ArmIOSim(), new ArmSolverIOKairos(1));
          cubeIntake = new CubeIntake(new CubeIntakeIOSim());
          objectiveTracker = new ObjectiveTracker(new NodeSelectorIOServer());
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
    if (arm == null) {
      arm = new Arm(new ArmIO() {}, new ArmSolverIO() {});
    }
    if (gripper == null) {
      gripper = new Gripper(new GripperIO() {});
    }
    if (cubeIntake == null) {
      cubeIntake = new CubeIntake(new CubeIntakeIO() {});
    }
    if (aprilTagVision == null) {
      // In replay, match the number of instances for each robot
      switch (Constants.getRobot()) {
        case ROBOT_2023P:
          aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
          break;
        default:
          aprilTagVision = new AprilTagVision();
          break;
      }
    }
    if (objectiveTracker == null) {
      objectiveTracker = new ObjectiveTracker(new NodeSelectorIO() {});
    }

    // Set up subsystems
    arm.setOverrides(
        () -> armDisable.getAsBoolean(),
        () -> armCoast.getAsBoolean(),
        () -> forcePregenPaths.getAsBoolean());
    gripper.setOverrides(() -> forceGripperEnable.getAsBoolean());
    cubeIntake.setSuppliers(arm::cubeIntakeShouldExtend, () -> armCoast.getAsBoolean());
    aprilTagVision.setDataInterface(drive::addVisionData);

    // Set up auto routines
    AutoCommands autoCommands =
        new AutoCommands(drive, arm, gripper, cubeIntake, autoSelector::getResponses);
    autoSelector.addRoutine(
        "Field: Score Mid Link", List.of(), autoCommands.fieldScoreLink(NodeLevel.MID));
    autoSelector.addRoutine(
        "Field: Score Hybrid Link", List.of(), autoCommands.fieldScoreLink(NodeLevel.HYBRID));
    autoSelector.addRoutine(
        "Side: Score Two, Grab, And Maybe Balance",
        List.of(
            new AutoQuestion(
                "Which side?",
                List.of(AutoQuestionResponse.FIELD_SIDE, AutoQuestionResponse.WALL_SIDE)),
            new AutoQuestion(
                "Which level?",
                List.of(
                    AutoQuestionResponse.HIGH,
                    AutoQuestionResponse.MID,
                    AutoQuestionResponse.HYBRID)),
            new AutoQuestion(
                "Balance?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO))),
        autoCommands.sideScoreTwoMaybeGrabMaybeBalance(true));
    autoSelector.addRoutine(
        "Side: Score Two And Maybe Balance",
        List.of(
            new AutoQuestion(
                "Which side?",
                List.of(AutoQuestionResponse.FIELD_SIDE, AutoQuestionResponse.WALL_SIDE)),
            new AutoQuestion(
                "Which level?",
                List.of(
                    AutoQuestionResponse.HIGH,
                    AutoQuestionResponse.MID,
                    AutoQuestionResponse.HYBRID)),
            new AutoQuestion(
                "Balance?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO))),
        autoCommands.sideScoreTwoMaybeGrabMaybeBalance(false));
    autoSelector.addRoutine(
        "Center: Score Two And Balance",
        List.of(
            new AutoQuestion(
                "Which side to intake from?",
                List.of(AutoQuestionResponse.FIELD_SIDE, AutoQuestionResponse.WALL_SIDE))),
        autoCommands.centerScoreTwoAndBalance());
    autoSelector.addRoutine(
        "Side: Score One And Maybe Balance",
        List.of(
            new AutoQuestion(
                "Which side?",
                List.of(AutoQuestionResponse.FIELD_SIDE, AutoQuestionResponse.WALL_SIDE)),
            new AutoQuestion(
                "Which level?",
                List.of(
                    AutoQuestionResponse.HIGH,
                    AutoQuestionResponse.MID,
                    AutoQuestionResponse.HYBRID)),
            new AutoQuestion(
                "Which node?",
                List.of(
                    AutoQuestionResponse.FIELD_SIDE,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.WALL_SIDE)),
            new AutoQuestion(
                "Balance?", List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO))),
        autoCommands.sideScoreOneAndMaybeBalance());
    autoSelector.addRoutine(
        "Center: Score One And Balance",
        List.of(
            new AutoQuestion(
                "Which level?",
                List.of(
                    AutoQuestionResponse.HIGH,
                    AutoQuestionResponse.MID,
                    AutoQuestionResponse.HYBRID)),
            new AutoQuestion(
                "Which node?",
                List.of(
                    AutoQuestionResponse.FIELD_SIDE,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.WALL_SIDE))),
        autoCommands.centerScoreOneAndBalance());
    autoSelector.addRoutine(
        "Drive Characterization",
        List.of(),
        new FeedForwardCharacterization(
            drive,
            true,
            new FeedForwardCharacterizationData("drive"),
            drive::runCharacterizationVolts,
            drive::getCharacterizationVelocity));

    // Startup alerts
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }
    if (FieldConstants.isWPIField) {
      new Alert("WPI field selected, do not use in competition.", AlertType.INFO).set(true);
    }

    // Endgame alert
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlertTime.get()))
        .onTrue(
            Commands.startEnd(
                    () -> {
                      Leds.getInstance().endgameAlert = true;
                      driver.getHID().setRumble(RumbleType.kBothRumble, 0.6);
                      operator.getHID().setRumble(RumbleType.kBothRumble, 0.6);
                    },
                    () -> {
                      Leds.getInstance().endgameAlert = false;
                      driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                      operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                    })
                .withTimeout(3.0));

    // Bind driver and operator controls
    bindControls();
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());
  }

  /** Updates the extra HP LED modes based on the override switch. */
  public void updateHPModeLeds() {
    Leds.getInstance().hpDoubleSubstation = hpDoubleSubstationSwitch.getAsBoolean();
    Leds.getInstance().hpThrowGamePiece = hpThrowGamePieceSwitch.getAsBoolean();
  }

  /** Binds the driver and operator controls. */
  public void bindControls() {
    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Joystick command factories
    Function<Boolean, DriveWithJoysticks> driveWithJoysticksFactory =
        (Boolean alwaysSniper) ->
            new DriveWithJoysticks(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> alwaysSniper || driver.getHID().getLeftBumper(),
                () -> robotRelative.getAsBoolean(),
                arm::getExtensionPercent);
    Supplier<MoveArmWithJoysticks> moveArmWithJoysticksFactory =
        () ->
            new MoveArmWithJoysticks(
                arm,
                () -> operator.getLeftX(),
                () -> -operator.getLeftY(),
                () -> -operator.getRightY());

    // *** DRIVER CONTROLS ***

    // Drive controls
    drive.setDefaultCommand(driveWithJoysticksFactory.apply(false));
    driver
        .start()
        .or(driver.back())
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              AllianceFlipUtil.apply(new Rotation2d())));
                    })
                .ignoringDisable(true));
    driver
        .b()
        .and(DriverStation::isDisabled)
        .whileTrue(
            Commands.run(
                    () -> {
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation().getX()
                                  + (driver.getLeftX() * Constants.loopPeriodSecs * 2.0),
                              drive.getPose().getTranslation().getY()
                                  - (driver.getLeftY() * Constants.loopPeriodSecs * 2.0),
                              drive
                                  .getRotation()
                                  .plus(
                                      new Rotation2d(
                                          -driver.getRightX() * Constants.loopPeriodSecs * 2.0))));
                    })
                .ignoringDisable(true));

    // Auto align controls
    driver
        .leftTrigger()
        .whileTrue(
            new DriveToSubstation(drive, () -> operator.getHID().getXButton())
                .deadlineWith(
                    Commands.startEnd(
                        () -> Leds.getInstance().autoSubstation = true,
                        () -> Leds.getInstance().autoSubstation = false)));
    var autoScoreTrigger = driver.rightTrigger();
    var ejectTrigger = driver.a();
    autoScoreTrigger.whileTrue(
        new AutoScore(
                drive,
                arm,
                gripper,
                objectiveTracker.objective,
                driveWithJoysticksFactory.apply(true),
                moveArmWithJoysticksFactory.get(),
                () -> ejectTrigger.getAsBoolean(),
                () -> manualDrive.getAsBoolean(),
                () -> autoEject.getAsBoolean(),
                () -> reachScoringEnable.getAsBoolean())
            .deadlineWith(
                Commands.startEnd(
                    () -> Leds.getInstance().autoScore = true,
                    () -> Leds.getInstance().autoScore = false)));

    // Distraction LEDs
    driver
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                    () -> Leds.getInstance().distraction = true,
                    () -> Leds.getInstance().distraction = false)
                .ignoringDisable(true));

    // Log marker
    driver
        .y()
        .whileTrue(
            Commands.startEnd(
                    () -> Logger.getInstance().recordOutput("LogMarker", true),
                    () -> Logger.getInstance().recordOutput("LogMarker", false))
                .ignoringDisable(true));

    // *** OPERATOR CONTROLS ***

    // Intake controls
    operator
        .a()
        .whileTrue(new IntakeSubstation(true, arm, drive, gripper, objectiveTracker.objective));
    operator
        .x()
        .whileTrue(new IntakeSubstation(false, arm, drive, gripper, objectiveTracker.objective));
    operator
        .rightTrigger()
        .whileTrue(new IntakeCubeHandoff(cubeIntake, arm, gripper, objectiveTracker.objective));
    operator.leftTrigger().whileTrue(new IntakeConeFloor(arm, gripper, objectiveTracker.objective));
    var ejectBackCommand =
        Commands.waitSeconds(DoublePressTracker.maxLengthSecs)
            .andThen(new EjectHeld(false, arm, gripper).withName("EjectHeld_Back"));
    operator.b().onTrue(ejectBackCommand);
    DoublePressTracker.createTrigger(operator.b())
        .onTrue(new EjectHeld(true, arm, gripper).withName("EjectHeld_Front"));

    // Manual arm controls
    new Trigger(
            () ->
                Math.abs(operator.getLeftX()) > DriveWithJoysticks.deadband
                    || Math.abs(operator.getLeftY()) > DriveWithJoysticks.deadband
                    || Math.abs(operator.getRightY()) > DriveWithJoysticks.deadband)
        .and(new Trigger(() -> arm.isTrajectoryFinished(false)))
        .and(autoScoreTrigger.negate())
        .whileTrue(moveArmWithJoysticksFactory.get());
    operator
        .leftStick()
        .and(autoScoreTrigger.negate())
        .onTrue(Commands.runOnce(() -> arm.runPath(ArmPose.Preset.HOMED), arm));
    operator.back().and(autoScoreTrigger.negate()).whileTrue(gripper.ejectCommand(EjectSpeed.FAST));
    operator.start().and(autoScoreTrigger.negate()).whileTrue(gripper.intakeCommand());

    // Objective tracking controls
    operator
        .leftBumper()
        .onTrue(Commands.runOnce(() -> Leds.getInstance().hpCone = true).ignoringDisable(true));
    operator
        .rightBumper()
        .onTrue(Commands.runOnce(() -> Leds.getInstance().hpCone = false).ignoringDisable(true));
    operator.y().onTrue(objectiveTracker.toggleConeOrientationCommand());
    operator.povUp().whileTrue(objectiveTracker.shiftNodeCommand(Direction.UP));
    operator.povRight().whileTrue(objectiveTracker.shiftNodeCommand(Direction.RIGHT));
    operator.povDown().whileTrue(objectiveTracker.shiftNodeCommand(Direction.DOWN));
    operator.povLeft().whileTrue(objectiveTracker.shiftNodeCommand(Direction.LEFT));
  }

  /** Passes the autonomous command to the {@link Robot} class. */
  public Command getAutonomousCommand() {
    return autoSelector.getCommand();
  }
}
