// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.List;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.commands.DriveTrajectory;
import org.littletonrobotics.frc2023.commands.DriveWithJoysticks;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization;
import org.littletonrobotics.frc2023.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import org.littletonrobotics.frc2023.commands.HoldPose;
import org.littletonrobotics.frc2023.oi.HandheldOI;
import org.littletonrobotics.frc2023.oi.OISelector;
import org.littletonrobotics.frc2023.oi.OverrideOI;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVision;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIO;
import org.littletonrobotics.frc2023.subsystems.apriltagvision.AprilTagVisionIONorthstar;
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
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Subsystems
  private Drive drive;
  private AprilTagVision aprilTagVision;

  // OI objects
  private OverrideOI overrideOI = new OverrideOI();
  private HandheldOI handheldOI = new HandheldOI();

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
          aprilTagVision = new AprilTagVision(new AprilTagVisionIONorthstar("northstar"));
          break;
        case ROBOT_SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
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

    // Set up subsystems
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> handheldOI.getLeftDriveX(),
            () -> handheldOI.getLeftDriveY(),
            () -> handheldOI.getRightDriveY(),
            () -> overrideOI.getRobotRelative()));
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

    // Instantiate OI classes and bind buttons
    updateOI();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    overrideOI = OISelector.findOverrideOI();
    handheldOI = OISelector.findHandheldOI();

    // *** DRIVER CONTROLS ***
    handheldOI
        .getResetGyro()
        .onTrue(
            new InstantCommand(
                    () -> {
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              AllianceFlipUtil.apply(new Rotation2d())));
                    })
                .ignoringDisable(true));
    var target =
        FieldConstants.aprilTags
            .get(2)
            .toPose2d()
            .transformBy(new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d()));
    handheldOI.getDriverAssist().whileTrue(new HoldPose(drive, target));

    // *** OPERATOR CONTROLS ***
  }

  /** Passes the autonomous command to the {@link Robot} class. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
