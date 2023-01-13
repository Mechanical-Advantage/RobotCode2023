// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.frc2023.util.trajectory.CustomHolonomicDriveController;
import org.littletonrobotics.frc2023.util.trajectory.CustomTrajectoryGenerator;
import org.littletonrobotics.frc2023.util.trajectory.RotationSequence;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory extends CommandBase {
  private static final Alert generatorAlert =
      new Alert("Failed to generate trajectory, check constants.", AlertType.ERROR);

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

  private final CustomHolonomicDriveController customHolonomicDriveController =
      new CustomHolonomicDriveController(xController, yController, thetaController);

  private final Drive drive;
  private final Timer timer = new Timer();

  private Supplier<List<Waypoint>> waypointsSupplier = null;
  private Supplier<List<TrajectoryConstraint>> constraintsSupplier = null;
  private CustomTrajectoryGenerator customGenerator = new CustomTrajectoryGenerator();

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023P:
        maxVelocityMetersPerSec = Units.inchesToMeters(150.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(150.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(100.0);

        driveKp.initDefault(2.5);
        driveKd.initDefault(0.0);
        turnKp.initDefault(7.0);
        turnKd.initDefault(0.0);
        break;
      case ROBOT_SIMBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(150.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(200.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(100.0);

        driveKp.initDefault(0.0);
        driveKd.initDefault(0.0);
        turnKp.initDefault(0.0);
        turnKd.initDefault(0.0);
        break;
      default:
        supportedRobot = false;
        break;
    }
  }

  /** Creates a DriveTrajectory command with a dynamic set of waypoints. */
  public DriveTrajectory(Drive drive, Supplier<List<Waypoint>> waypointsSupplier) {
    this(drive, waypointsSupplier, () -> List.of());
  }

  /** Creates a DriveTrajectory command with a dynamic set of waypoints and constraints. */
  public DriveTrajectory(
      Drive drive,
      Supplier<List<Waypoint>> waypointsSupplier,
      Supplier<List<TrajectoryConstraint>> constraintsSupplier) {
    this.drive = drive;
    addRequirements(drive);
    this.waypointsSupplier = waypointsSupplier;
    this.constraintsSupplier = constraintsSupplier;
  }

  /** Creates a DriveTrajectory command with a static set of waypoints. */
  public DriveTrajectory(Drive drive, List<Waypoint> waypoints) {
    this(drive, waypoints, List.of());
  }

  /** Creates a DriveTrajectory command with a static set of waypoints and constraints. */
  public DriveTrajectory(
      Drive drive, List<Waypoint> waypoints, List<TrajectoryConstraint> constraints) {
    this.drive = drive;
    addRequirements(drive);
    generate(waypoints, constraints);
  }

  /** Generates the trajectory. */
  private void generate(List<Waypoint> waypoints, List<TrajectoryConstraint> constraints) {
    // Set up trajectory configuration
    TrajectoryConfig config =
        new TrajectoryConfig(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2)
            .setKinematics(new SwerveDriveKinematics(drive.getModuleTranslations()))
            .setStartVelocity(0.0)
            .setEndVelocity(0.0)
            .addConstraint(
                new CentripetalAccelerationConstraint(maxCentripetalAccelerationMetersPerSec2))
            .addConstraints(constraints);

    // Generate trajectory
    customGenerator = new CustomTrajectoryGenerator(); // Reset generator
    try {
      customGenerator.generate(config, waypoints);
    } catch (TrajectoryGenerationException exception) {
      if (supportedRobot) {
        generatorAlert.set(true);
        DriverStation.reportError("Failed to generate trajectory.", true);
      }
    }
  }

  @Override
  public void initialize() {
    // Generate trajectory if supplied
    if (waypointsSupplier != null || constraintsSupplier != null) {
      generate(waypointsSupplier.get(), constraintsSupplier.get());
    }

    // Log trajectory
    Logger.getInstance().recordOutput("Odometry/Trajectory", customGenerator.getDriveTrajectory());

    // Reset all controllers
    timer.reset();
    timer.start();
    xController.reset();
    yController.reset();
    thetaController.reset();

    // Reset PID gains
    xController.setD(driveKd.get());
    xController.setP(driveKp.get());
    yController.setD(driveKd.get());
    yController.setP(driveKp.get());
    thetaController.setD(driveKd.get());
    thetaController.setP(driveKp.get());
  }

  @Override
  public void execute() {
    // Update from tunable numbers
    if (driveKd.hasChanged(hashCode())
        || driveKp.hasChanged(hashCode())
        || turnKd.hasChanged(hashCode())
        || turnKp.hasChanged(hashCode())) {
      xController.setD(driveKd.get());
      xController.setP(driveKp.get());

      yController.setD(driveKd.get());
      yController.setP(driveKp.get());

      thetaController.setD(turnKd.get());
      thetaController.setP(turnKp.get());
    }

    // Get setpoint
    Trajectory.State driveState =
        AllianceFlipUtil.apply(customGenerator.getDriveTrajectory().sample(timer.get()));
    RotationSequence.State holonomicRotationState =
        AllianceFlipUtil.apply(customGenerator.getHolonomicRotationSequence().sample(timer.get()));
    Logger.getInstance()
        .recordOutput(
            "Odometry/TrajectorySetpoint",
            new double[] {
              driveState.poseMeters.getX(),
              driveState.poseMeters.getY(),
              holonomicRotationState.position.getRadians()
            });

    // Calculate velocity
    ChassisSpeeds nextDriveState =
        customHolonomicDriveController.calculate(
            drive.getPose(), driveState, holonomicRotationState);
    drive.runVelocity(nextDriveState);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(customGenerator.getDriveTrajectory().getTotalTimeSeconds());
  }
}
