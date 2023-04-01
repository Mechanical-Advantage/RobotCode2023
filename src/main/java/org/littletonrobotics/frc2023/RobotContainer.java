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
import org.littletonrobotics.frc2023.commands.HoldFlippableArmPreset;
import org.littletonrobotics.frc2023.commands.IntakeConeFloor;
import org.littletonrobotics.frc2023.commands.IntakeCubeHandoff;
import org.littletonrobotics.frc2023.commands.IntakeSubstation;
import org.littletonrobotics.frc2023.commands.MoveArmWithJoysticks;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIONorthstar;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.Arm.LiftDirection;
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
import org.littletonrobotics.frc2023.subsystems.leds.Leds.HPGamePiece;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIOServer;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Direction;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.DoublePressTracker;
import org.littletonrobotics.frc2023.util.OverrideSwitches;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.TriggerUtil;
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
  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 15.0);

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
        case ROBOT_2023C:
          aprilTagVision =
              new AprilTagVision(
                  new AprilTagVisionIO() {},
                  new AprilTagVisionIO() {},
                  new AprilTagVisionIO() {},
                  new AprilTagVisionIO() {});
          break;
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
        "Drive and Balance",
        List.of(),
        Commands.runOnce(() -> drive.setPose(autoCommands.startingLocations[4]))
            .andThen(autoCommands.driveAndBalance(autoCommands.startingLocations[4], false)));
    autoSelector.addRoutine(
        "Field: Score Three Combo", List.of(), autoCommands.fieldScoreThreeCombo());
    autoSelector.addRoutine(
        "Field: Score Link",
        List.of(
            new AutoQuestion(
                "Which level?",
                List.of(
                    AutoQuestionResponse.HIGH,
                    AutoQuestionResponse.MID,
                    AutoQuestionResponse.HYBRID))),
        autoCommands.fieldScoreLink());
    autoSelector.addRoutine(
        "Field: Score Two, Grab, And Maybe Balance",
        List.of(
            new AutoQuestion(
                "Which level?",
                List.of(
                    AutoQuestionResponse.HIGH,
                    AutoQuestionResponse.MID,
                    AutoQuestionResponse.HYBRID)),
            new AutoQuestion(
                "End behavior?",
                List.of(
                    AutoQuestionResponse.RETURN,
                    AutoQuestionResponse.BALANCE,
                    AutoQuestionResponse.BALANCE_THROW))),
        autoCommands.sideScoreTwoGrabMaybeBalance());
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
        autoCommands.sideScoreTwoMaybeBalance());
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
        "Center: Score One, Grab, Balance, And Maybe Score",
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
                    AutoQuestionResponse.WALL_SIDE)),
            new AutoQuestion(
                "Which side to intake from?",
                List.of(AutoQuestionResponse.FIELD_SIDE, AutoQuestionResponse.WALL_SIDE)),
            new AutoQuestion(
                "Score cube while balanced?",
                List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO))),
        autoCommands.centerScoreOneGrabAndBalance());
    autoSelector.addRoutine(
        "Center: Score One, Mobility, And Balance",
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
        autoCommands.centerScoreOneMobilityAndBalance());
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
        "Reach for Inspection",
        List.of(),
        arm.runPathCommand(ArmPose.Preset.SCORE_HIGH_UPRIGHT_CONE));
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

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.run(
                    () -> {
                      Leds.getInstance().endgameAlert = true;
                      driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                      operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                    })
                .withTimeout(1.5)
                .andThen(
                    Commands.run(
                            () -> {
                              Leds.getInstance().endgameAlert = false;
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
                          Leds.getInstance().endgameAlert = true;
                          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          Leds.getInstance().endgameAlert = false;
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          Leds.getInstance().endgameAlert = true;
                          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          Leds.getInstance().endgameAlert = false;
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(1.0)));

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
        (Boolean sniper) ->
            new DriveWithJoysticks(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> sniper,
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
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Auto align controls
    Command intakeSubstationSingle =
        new IntakeSubstation(true, arm, drive, gripper, objectiveTracker.objective);
    Command intakeSubstationDouble =
        new IntakeSubstation(false, arm, drive, gripper, objectiveTracker.objective);
    driver
        .leftTrigger()
        .whileTrue(
            new DriveToSubstation(drive, () -> operator.getHID().getXButton())
                .deadlineWith(
                    Commands.startEnd(
                        () -> Leds.getInstance().autoSubstation = true,
                        () -> Leds.getInstance().autoSubstation = false))
                .withName("DriveToSubstation"))
        .onFalse(
            Commands.runOnce(
                () -> {
                  // Cancel intaking, forcing the gripper to restart
                  intakeSubstationSingle.cancel();
                  intakeSubstationDouble.cancel();
                }));
    var autoScoreTrigger = driver.rightTrigger();
    var ejectTrigger = driver.a();
    autoScoreTrigger
        .whileTrue(
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
                        () -> Leds.getInstance().autoScore = false)))
        .onTrue(Commands.runOnce(() -> Leds.getInstance().hpGamePiece = HPGamePiece.NONE));

    // Throw game piece
    driver
        .leftBumper()
        .whileTrue(
            new HoldFlippableArmPreset(
                arm, drive, ArmPose.Preset.THROW.getPose(), Rotation2d.fromDegrees(180.0)));
    driver.leftBumper().onFalse(gripper.ejectCommand(EjectSpeed.VERY_FAST));

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

    // Arm lift controls
    driver
        .povRight()
        .whileTrue(
            Commands.startEnd(
                () -> arm.setEmergencyLiftDirection(LiftDirection.FRONT),
                () -> {
                  arm.setEmergencyLiftDirection(LiftDirection.NONE);
                  arm.runPath(ArmPose.Preset.HOMED);
                },
                arm));
    driver
        .povLeft()
        .whileTrue(
            Commands.startEnd(
                () -> arm.setEmergencyLiftDirection(LiftDirection.BACK),
                () -> {
                  arm.setEmergencyLiftDirection(LiftDirection.NONE);
                  arm.runPath(ArmPose.Preset.HOMED);
                },
                arm));

    // *** OPERATOR CONTROLS ***

    // Intake controls
    TriggerUtil.whileTrueContinuous(
        operator.a().and(operator.x().negate()), intakeSubstationSingle);
    TriggerUtil.whileTrueContinuous(
        operator.x().and(operator.a().negate()), intakeSubstationDouble);
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
                Math.abs(operator.getLeftX()) > DriveWithJoysticks.deadband.get()
                    || Math.abs(operator.getLeftY()) > DriveWithJoysticks.deadband.get()
                    || Math.abs(operator.getRightY()) > DriveWithJoysticks.deadband.get())
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
        .onTrue(
            Commands.runOnce(() -> Leds.getInstance().hpGamePiece = HPGamePiece.CONE)
                .ignoringDisable(true));
    operator
        .rightBumper()
        .onTrue(
            Commands.runOnce(() -> Leds.getInstance().hpGamePiece = HPGamePiece.CUBE)
                .ignoringDisable(true));
    new Trigger(DriverStation::isEnabled)
        .onTrue(Commands.runOnce(() -> Leds.getInstance().hpGamePiece = HPGamePiece.NONE));
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
