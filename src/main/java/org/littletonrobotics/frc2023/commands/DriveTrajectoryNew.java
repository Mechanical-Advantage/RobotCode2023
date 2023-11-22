// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import static org.littletonrobotics.frc2023.util.trajectory.updated.CustomSwerveDriveController.DriveDynamicState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.frc2023.util.trajectory.updated.CustomSwerveDriveController;
import org.littletonrobotics.frc2023.util.trajectory.updated.TrajectoryImplementation;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectoryNew extends CommandBase {
  private static final Alert generatorAlert =
      new Alert("Failed to generate trajectory, check constants.", Alert.AlertType.ERROR);

  private static boolean supportedRobot = true;
  private static double maxVelocityMetersPerSec;
  private static double maxAccelerationMetersPerSec2;
  private static double maxCentripetalAccelerationMetersPerSec2;

  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("DriveTrajectory/DriveKp");
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber("DriveTrajectory/DriveKd");
  private static final LoggedTunableNumber turnKp =
      new LoggedTunableNumber("DriveTrajectory/TurnKp");
  private static final LoggedTunableNumber turnKd =
      new LoggedTunableNumber("DriveTrajectory/TurnKd");

  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

  private CustomSwerveDriveController driveController =
      new CustomSwerveDriveController(xController, yController, thetaController);

  private final Drive drive;
  private TrajectoryImplementation trajectory;
  private final Timer timer = new Timer();

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
      case ROBOT_2023P:
        maxVelocityMetersPerSec = Units.inchesToMeters(140.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(80.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(130.0);

        driveKp.initDefault(6.0);
        driveKd.initDefault(0.0);
        turnKp.initDefault(8.0);
        turnKd.initDefault(0.0);
        break;
      case ROBOT_SIMBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(140.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(80.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(130.0);

        driveKp.initDefault(2.5);
        driveKd.initDefault(0.0);
        turnKp.initDefault(7.0);
        turnKd.initDefault(0.0);
        break;
      default:
        supportedRobot = false;
        break;
    }
  }

  public DriveTrajectoryNew(Drive drive, TrajectoryImplementation trajectory) {
    this.drive = drive;
    this.trajectory = trajectory;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Log trajectory
    Logger.getInstance().recordOutput("Odometry/Trajectory", trajectory.getTrajectoryPoses());

    // Reset all controllers
    timer.reset();
    timer.start();
    xController.reset();
    yController.reset();
    thetaController.reset();

    // Reset PID gains
    xController.setP(driveKp.get());
    xController.setD(driveKd.get());
    yController.setP(driveKp.get());
    yController.setD(driveKd.get());
    thetaController.setP(turnKp.get());
    thetaController.setD(turnKd.get());
  }

  @Override
  public void execute() {
    // Update from tunable numbers
    if (driveKd.hasChanged(hashCode())
        || driveKp.hasChanged(hashCode())
        || turnKd.hasChanged(hashCode())
        || turnKp.hasChanged(hashCode())) {
      xController.setP(driveKp.get());
      xController.setD(driveKd.get());
      yController.setP(driveKp.get());
      yController.setD(driveKd.get());
      thetaController.setP(turnKp.get());
      thetaController.setD(turnKd.get());
    }

    DriveDynamicState state = trajectory.sample(timer.get()).maybeFlip();
    Pose2d statePose = state.pose();
    Logger.getInstance().recordOutput("Odometry/TrajectorySetpoint", statePose);
    ChassisSpeeds nextDriveState = driveController.calculate(drive.getPose(), state);

    Logger.getInstance()
        .recordOutput(
            "Drive/ffSpeeds",
            new double[] {state.velocityX(), state.velocityY(), state.angularVelocity()});
    Logger.getInstance()
        .recordOutput(
            "Drive/autoSpeeds",
            new double[] {
              nextDriveState.vxMetersPerSecond,
              nextDriveState.vyMetersPerSecond,
              nextDriveState.omegaRadiansPerSecond
            });
    drive.runVelocity(nextDriveState);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getDuration());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.getInstance().recordOutput("Odometry/Trajectory", new double[] {});
    Logger.getInstance().recordOutput("Odometry/TrajectorySetpoint", new double[] {});
  }
}
