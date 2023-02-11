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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.commands.DriveToNode;
import org.littletonrobotics.frc2023.commands.DriveToSubstation;
import org.littletonrobotics.frc2023.commands.DriveTrajectory;
import org.littletonrobotics.frc2023.commands.DriveWithJoysticks;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import org.littletonrobotics.frc2023.commands.HoldFlippableArmPreset;
import org.littletonrobotics.frc2023.commands.RaiseArmToScore;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO;
import org.littletonrobotics.frc2023.subsystems.arm.Arm;
import org.littletonrobotics.frc2023.subsystems.arm.Arm.ArmPose;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIO;
import org.littletonrobotics.frc2023.subsystems.arm.ArmIOSim;
import org.littletonrobotics.frc2023.subsystems.arm.ArmSolverIO;
import org.littletonrobotics.frc2023.subsystems.arm.ArmSolverIOKairos;
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
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Subsystems
  private Drive drive;
  private Arm arm;
  private Gripper gripper;
  private CubeIntake cubeIntake;
  private AprilTagVision aprilTagVision;
  private ObjectiveTracker objectiveTracker;

  // OI objects
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);
  private OverrideSwitches overrides = new OverrideSwitches(5);
  private Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.INFO);

  // Choosers
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public RobotContainer() {
    // Check if flash should be burned
    SparkMaxBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
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
    cubeIntake.setForceExtendSupplier(arm::cubeIntakeShouldExtend);
    aprilTagVision.setDataInterfaces(drive::getPose, drive::addVisionData);

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", null);
    autoChooser.addOption("Reset Odometry", new InstantCommand(() -> drive.setPose(new Pose2d())));
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drive,
            true,
            new FeedForwardCharacterizationData("drive"),
            drive::runCharacterizationVolts,
            drive::getCharacterizationVelocity));

    autoChooser.addOption(
        "Test Trajectory",
        new InstantCommand(() -> drive.setPose(new Pose2d()))
            .andThen(
                new DriveTrajectory(
                    drive,
                    List.of(
                        Waypoint.fromHolonomicPose(new Pose2d()),
                        Waypoint.fromHolonomicPose(
                            new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(45.0)))))));

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
            () -> overrides.getDriverSwitch(0)));
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

    // Auto align controls
    driver.leftTrigger().whileTrue(new DriveToSubstation(drive));
    var driveToNode = new DriveToNode(drive, objectiveTracker);
    driver
        .y()
        .whileTrue(
            Commands.parallel(
                    new RaiseArmToScore(arm, drive, objectiveTracker),
                    new WaitUntilCommand(driveToNode::atGoal))
                .andThen(gripper.ejectCommand())
                .deadlineWith(driveToNode)
                .finallyDo((interrupted) -> arm.runPath(ArmPose.Preset.HOMED)));

    // *** OPERATOR CONTROLS ***

    // Objective tracking controls
    operator
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> objectiveTracker.gamePiece = GamePiece.CONE)
                .ignoringDisable(true));
    operator
        .rightBumper()
        .onTrue(
            Commands.runOnce(() -> objectiveTracker.gamePiece = GamePiece.CUBE)
                .ignoringDisable(true));
    operator.povUp().whileTrue(objectiveTracker.shiftNodeCommand(Direction.UP));
    operator.povRight().whileTrue(objectiveTracker.shiftNodeCommand(Direction.RIGHT));
    operator.povDown().whileTrue(objectiveTracker.shiftNodeCommand(Direction.DOWN));
    operator.povLeft().whileTrue(objectiveTracker.shiftNodeCommand(Direction.LEFT));

    // Intake controls
    var singleSubstationArmCommand =
        new HoldFlippableArmPreset(
            arm, drive, ArmPose.Preset.SINGLE_SUBTATION.getPose(), Rotation2d.fromDegrees(90.0));
    operator
        .a()
        .whileTrue(
            singleSubstationArmCommand.alongWith(
                gripper.intakeCommand(),
                Commands.run(
                    () ->
                        objectiveTracker.lastIntakeFront =
                            !singleSubstationArmCommand.isFlipped())));
    var doubleSubstationArmCommand =
        new HoldFlippableArmPreset(
            arm, drive, ArmPose.Preset.DOUBLE_SUBTATION.getPose(), Rotation2d.fromDegrees(0.0));
    operator
        .b()
        .whileTrue(
            doubleSubstationArmCommand.alongWith(
                gripper.intakeCommand(),
                Commands.run(
                    () ->
                        objectiveTracker.lastIntakeFront =
                            !doubleSubstationArmCommand.isFlipped())));
  }

  /** Passes the autonomous command to the {@link Robot} class. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
