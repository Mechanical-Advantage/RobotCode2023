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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
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
import org.littletonrobotics.frc2023.commands.IntakeCubeHandoff;
import org.littletonrobotics.frc2023.commands.IntakeFromFloorSimple;
import org.littletonrobotics.frc2023.commands.IntakeSubstation;
import org.littletonrobotics.frc2023.commands.MoveArmWithJoysticks;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIO;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIOSim;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIOSparkMax;
import org.littletonrobotics.frc2023.subsystems.arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.arm.ArmSolverIO;
import org.littletonrobotics.frc2023.subsystems.arm.ArmSolverIOKairos;
import org.littletonrobotics.frc2023.subsystems.coneintake.ConeIntake;
import org.littletonrobotics.frc2023.subsystems.coneintake.ConeIntakeIO;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntake;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntakeIO;
import org.littletonrobotics.frc2023.subsystems.cubeintake.CubeIntakeIOSim;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.drive.GyroIO;
import org.littletonrobotics.frc2023.subsystems.drive.GyroIOPigeon2;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIO;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.frc2023.subsystems.gripper.Gripper;
import org.littletonrobotics.frc2023.subsystems.gripper.GripperIO;
import org.littletonrobotics.frc2023.subsystems.gripper.GripperIOSparkMax;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIO;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.NodeSelectorIOServer;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.Direction;
import org.littletonrobotics.frc2023.subsystems.objectivetracker.ObjectiveTracker.GamePiece;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.OverrideSwitches;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class RobotContainer {

  // Subsystems
  private Drive drive;
  private Arm arm;
  private Gripper gripper;
  private CubeIntake cubeIntake;
  private ConeIntake coneIntake;
  private AprilTagVision aprilTagVision;
  private ObjectiveTracker objectiveTracker;

  // OI objects
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger robotRelativeOverride = overrides.driverSwitch(0);
  private final Trigger armDisableOverride = overrides.driverSwitch(1);
  private final Trigger armCoastOverride = overrides.driverSwitch(2);
  private final Trigger manualDriveAlignOverride = overrides.operatorSwitch(0);
  private final Trigger manualArmAdjustOverride = overrides.operatorSwitch(1);
  private final Trigger reachScoringDisableOverride = overrides.operatorSwitch(2);
  private final Trigger forcePregenPathsOverride = overrides.operatorSwitch(3);
  private final Trigger floorEjectOverride = overrides.operatorSwitch(4);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.INFO);

  // Auto selector
  private final AutoSelector autoSelector = new AutoSelector("Auto");

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
          arm = new Arm(new ArmIOSparkMax(), new ArmSolverIOKairos(1));
          gripper = new Gripper(new GripperIOSparkMax());
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
    if (coneIntake == null) {
      coneIntake = new ConeIntake(new ConeIntakeIO() {});
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
        () -> armDisableOverride.getAsBoolean(),
        () -> armCoastOverride.getAsBoolean(),
        () -> forcePregenPathsOverride.getAsBoolean());
    cubeIntake.setForceExtendSupplier(arm::cubeIntakeShouldExtend);
    coneIntake.setForceExtendSupplier(arm::coneIntakeShouldExtend);
    aprilTagVision.setDataInterfaces(drive::getPose, drive::addVisionData);

    // Set up auto routines
    AutoCommands autoCommands =
        new AutoCommands(drive, arm, gripper, cubeIntake, coneIntake, autoSelector::getResponses);
    autoSelector.addRoutine(
        "Reset Odometry", List.of(), new InstantCommand(() -> drive.setPose(new Pose2d())));
    autoSelector.addRoutine(
        "Score Link",
        List.of(
            new AutoQuestion(
                "Which side of the field?",
                List.of(AutoQuestionResponse.WALL_SIDE, AutoQuestionResponse.FIELD_SIDE)),
            new AutoQuestion(
                "Which level?",
                List.of(
                    AutoQuestionResponse.HIGH,
                    AutoQuestionResponse.MID,
                    AutoQuestionResponse.HYBRID)),
            new AutoQuestion(
                "Finish with balance?",
                List.of(AutoQuestionResponse.YES, AutoQuestionResponse.NO))),
        autoCommands.scoreLink());
    autoSelector.addRoutine(
        "Drive Characterization",
        List.of(),
        new FeedForwardCharacterization(
            drive,
            true,
            new FeedForwardCharacterizationData("drive"),
            drive::runCharacterizationVolts,
            drive::getCharacterizationVelocity));

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }

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

  /** Binds the driver and operator controls. */
  public void bindControls() {
    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // *** DRIVER CONTROLS ***

    // Drive controls
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> robotRelativeOverride.getAsBoolean(),
            arm::getExtensionPercent));
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
    driver.leftTrigger().whileTrue(new DriveToSubstation(drive));
    var autoScoreTrigger = driver.rightTrigger();
    var operatorEjectTrigger = operator.back();
    Supplier<MoveArmWithJoysticks> moveArmWithJoysticksFactory =
        () ->
            new MoveArmWithJoysticks(
                arm,
                () -> operator.getLeftX(),
                () -> -operator.getLeftY(),
                () -> -operator.getRightY());

    // Full auto mode (auto drive, auto arm)
    autoScoreTrigger
        .and(manualDriveAlignOverride.negate())
        .and(manualArmAdjustOverride.negate())
        .whileTrue(
            new AutoScore(
                drive,
                arm,
                gripper,
                objectiveTracker.objective,
                () -> reachScoringDisableOverride.getAsBoolean()));

    // Semi-auto (manual drive, auto arm)
    autoScoreTrigger
        .and(manualDriveAlignOverride)
        .and(manualArmAdjustOverride.negate())
        .whileTrue(
            new AutoScore(
                drive::getPose,
                arm,
                gripper,
                objectiveTracker.objective,
                () -> reachScoringDisableOverride.getAsBoolean(),
                () -> operatorEjectTrigger.getAsBoolean()));

    // Semi-auto (auto drive, manual arm)
    autoScoreTrigger
        .and(manualDriveAlignOverride.negate())
        .and(manualArmAdjustOverride)
        .whileTrue(
            new AutoScore(
                drive,
                arm,
                gripper,
                objectiveTracker.objective,
                () -> reachScoringDisableOverride.getAsBoolean(),
                () -> operatorEjectTrigger.getAsBoolean(),
                moveArmWithJoysticksFactory.get()));

    // Manual (manual drive, manual arm)
    autoScoreTrigger
        .and(manualDriveAlignOverride)
        .and(manualArmAdjustOverride)
        .whileTrue(
            new AutoScore(
                drive::getPose,
                arm,
                gripper,
                objectiveTracker.objective,
                () -> reachScoringDisableOverride.getAsBoolean(),
                () -> operatorEjectTrigger.getAsBoolean(),
                moveArmWithJoysticksFactory.get()));

    // *** OPERATOR CONTROLS ***

    // Intake controls
    operator
        .a()
        .whileTrue(new IntakeSubstation(true, arm, drive, gripper, objectiveTracker.objective));
    operator
        .x()
        .whileTrue(new IntakeSubstation(false, arm, drive, gripper, objectiveTracker.objective));
    operator
        .b()
        // .and(() -> objectiveTracker.objective.gamePiece == GamePiece.CUBE)
        .whileTrue(new IntakeCubeHandoff(cubeIntake, arm, gripper, objectiveTracker.objective));
    // var coneIntakeTrigger =
    //     operator.b().and(() -> objectiveTracker.objective.gamePiece == GamePiece.CONE);
    // coneIntakeTrigger.onTrue(
    //     new IntakeConeHandoff(
    //         coneIntake, arm, gripper, objectiveTracker.objective,
    // coneIntakeTrigger::getAsBoolean));
    operator
        .rightTrigger()
        .and(floorEjectOverride.negate())
        .whileTrue(new IntakeFromFloorSimple(true, arm, gripper, objectiveTracker.objective));
    operator
        .leftTrigger()
        .and(floorEjectOverride.negate())
        .whileTrue(new IntakeFromFloorSimple(false, arm, gripper, objectiveTracker.objective));
    operator.rightTrigger().and(floorEjectOverride).onTrue(new EjectHeld(true, arm, gripper));
    operator.leftTrigger().and(floorEjectOverride).whileTrue(new EjectHeld(false, arm, gripper));

    // Manual arm controls
    new Trigger(
            () ->
                Math.abs(operator.getLeftX()) > DriveWithJoysticks.deadband
                    || Math.abs(operator.getLeftY()) > DriveWithJoysticks.deadband
                    || Math.abs(operator.getRightY()) > DriveWithJoysticks.deadband)
        .and(new Trigger(() -> arm.isTrajectoryFinished()))
        .and(autoScoreTrigger.negate())
        .whileTrue(moveArmWithJoysticksFactory.get());
    operator
        .leftStick()
        .and(autoScoreTrigger.negate())
        .onTrue(Commands.runOnce(() -> arm.runPath(ArmPose.Preset.HOMED), arm));
    operatorEjectTrigger // Trigger defined above b/c used for auto scoring
        .and(autoScoreTrigger.negate())
        .whileTrue(gripper.ejectCommand());
    operator.start().and(autoScoreTrigger.negate()).whileTrue(gripper.intakeCommand());

    // Objective tracking controls
    operator
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> objectiveTracker.objective.gamePiece = GamePiece.CONE)
                .ignoringDisable(true));
    operator
        .rightBumper()
        .onTrue(
            Commands.runOnce(() -> objectiveTracker.objective.gamePiece = GamePiece.CUBE)
                .ignoringDisable(true));
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
