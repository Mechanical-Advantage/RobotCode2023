// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.leds.Leds;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.frc2023.util.PoseEstimator;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double coastThresholdMetersPerSec =
      0.05; // Need to be under this to switch to coast when disabling
  private static final double coastThresholdSecs =
      6.0; // Need to be under the above speed for this length of time to switch to coast
  private static final double ledsFallenAngleDegrees = 60.0; // Threshold to detect falls

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private static final LoggedTunableNumber maxLinearSpeed =
      new LoggedTunableNumber("Drive/MaxLinearSpeed");
  private static final LoggedTunableNumber trackWidthX =
      new LoggedTunableNumber("Drive/TrackWidthX");
  private static final LoggedTunableNumber trackWidthY =
      new LoggedTunableNumber("Drive/TrackWidthY");

  private double maxAngularSpeed;
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  private boolean isCharacterizing = false;
  private ChassisSpeeds setpoint = new ChassisSpeeds();
  private SwerveModuleState[] lastSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private double characterizationVolts = 0.0;
  private boolean isBrakeMode = false;
  private Timer lastMovementTimer = new Timer();

  private PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
  private Rotation2d lastGyroYaw = new Rotation2d();
  private Twist2d fieldVelocity = new Twist2d();

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
      case ROBOT_SIMBOT:
        maxLinearSpeed.initDefault(Units.feetToMeters(14.5));
        trackWidthX.initDefault(Units.inchesToMeters(19.75));
        trackWidthY.initDefault(Units.inchesToMeters(19.75));
      case ROBOT_2023P:
        maxLinearSpeed.initDefault(Units.feetToMeters(14.5));
        trackWidthX.initDefault(Units.inchesToMeters(25.0));
        trackWidthY.initDefault(Units.inchesToMeters(24.0));
        break;
      default:
        break;
    }
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    System.out.println("[Init] Creating Drive");
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    lastMovementTimer.start();
    for (var module : modules) {
      module.setBrakeMode(false);
    }
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Update if tunable numbers have changed
    if (maxLinearSpeed.hasChanged(hashCode())
        || trackWidthX.hasChanged(hashCode())
        || trackWidthY.hasChanged(hashCode())) {
      kinematics = new SwerveDriveKinematics(getModuleTranslations());
      maxAngularSpeed =
          maxLinearSpeed.get()
              / Arrays.stream(getModuleTranslations())
                  .map(translation -> translation.getNorm())
                  .max(Double::compare)
                  .get();
    }

    // Run modules
    if (DriverStation.isDisabled()) {
      // Stop moving while disabled
      for (var module : modules) {
        module.stop();
      }

      // Clear setpoint logs
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

    } else if (isCharacterizing) {
      // Run in characterization mode
      for (var module : modules) {
        module.runCharacterization(characterizationVolts);
      }

      // Clear setpoint logs
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});

    } else {
      // Calculate module setpoints
      var setpointTwist =
          new Pose2d()
              .log(
                  new Pose2d(
                      setpoint.vxMetersPerSecond * Constants.loopPeriodSecs,
                      setpoint.vyMetersPerSecond * Constants.loopPeriodSecs,
                      new Rotation2d(setpoint.omegaRadiansPerSecond * Constants.loopPeriodSecs)));
      var adjustedSpeeds =
          new ChassisSpeeds(
              setpointTwist.dx / Constants.loopPeriodSecs,
              setpointTwist.dy / Constants.loopPeriodSecs,
              setpointTwist.dtheta / Constants.loopPeriodSecs);
      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed.get());

      // Set to last angles if zero
      if (adjustedSpeeds.vxMetersPerSecond == 0.0
          && adjustedSpeeds.vyMetersPerSecond == 0.0
          && adjustedSpeeds.omegaRadiansPerSecond == 0) {
        for (int i = 0; i < 4; i++) {
          setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
        }
      }
      lastSetpointStates = setpointStates;

      // Send setpoints to modules
      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
      }

      // Log setpoint states
      Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
    }

    // Log measured states
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }
    Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);

    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
              modules[i].getAngle());
      lastModulePositionsMeters[i] = modules[i].getPositionMeters();
    }
    var twist = kinematics.toTwist2d(wheelDeltas);
    var gyroYaw = new Rotation2d(gyroInputs.yawPositionRad);
    if (gyroInputs.connected) {
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
    }
    lastGyroYaw = gyroYaw;
    poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
    Logger.getInstance().recordOutput("Odometry/Robot", getPose());

    // Log 3D odometry pose
    Pose3d robotPose3d = new Pose3d(getPose());
    robotPose3d =
        robotPose3d
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroInputs.pitchPositionRad) * trackWidthX.get() / 2.0,
                    0.0,
                    gyroInputs.pitchPositionRad,
                    0.0))
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroInputs.rollPositionRad) * trackWidthY.get() / 2.0,
                    gyroInputs.rollPositionRad,
                    0.0,
                    0.0));
    Logger.getInstance().recordOutput("Odometry/Robot3d", robotPose3d);

    // Update field velocity
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
    Translation2d linearFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());
    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : chassisSpeeds.omegaRadiansPerSecond);

    // Update brake mode
    boolean stillMoving = false;
    for (int i = 0; i < 4; i++) {
      if (Math.abs(modules[i].getVelocityMetersPerSec()) > coastThresholdMetersPerSec) {
        stillMoving = true;
      }
    }
    if (stillMoving) lastMovementTimer.reset();
    if (DriverStation.isEnabled()) {
      if (!isBrakeMode) {
        isBrakeMode = true;
        for (var module : modules) {
          module.setBrakeMode(true);
        }
      }
    } else {
      if (isBrakeMode && lastMovementTimer.hasElapsed(coastThresholdSecs)) {
        isBrakeMode = false;
        for (var module : modules) {
          module.setBrakeMode(false);
        }
      }
    }

    // Check for fallen robot
    Leds.getInstance().fallen =
        Units.radiansToDegrees(Math.abs(gyroInputs.pitchPositionRad)) > ledsFallenAngleDegrees
            || Units.radiansToDegrees(Math.abs(gyroInputs.rollPositionRad))
                > ledsFallenAngleDegrees;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    isCharacterizing = false;
    setpoint = speeds;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    stop();
    for (int i = 0; i < 4; i++) {
      lastSetpointStates[i] =
          new SwerveModuleState(
              lastSetpointStates[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle());
    }
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxLinearSpeed.get();
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxAngularSpeed;
  }

  /**
   * Returns the measured X, Y, and theta field velocities in meters per sec. The components of the
   * twist are velocities and NOT changes in position.
   */
  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  /** Returns the current pitch (Y rotation). */
  public Rotation2d getPitch() {
    return new Rotation2d(gyroInputs.pitchPositionRad);
  }

  /** Returns the current roll (X rotation). */
  public Rotation2d getRoll() {
    return new Rotation2d(gyroInputs.rollPositionRad);
  }

  /** Returns the current yaw velocity (Z rotation) in radians per second. */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /** Returns the current pitch velocity (Y rotation) in radians per second. */
  public double getPitchVelocity() {
    return gyroInputs.pitchVelocityRadPerSec;
  }

  /** Returns the current roll velocity (X rotation) in radians per second. */
  public double getRollVelocity() {
    return gyroInputs.rollVelocityRadPerSec;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    return poseEstimator.getLatestPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return poseEstimator.getLatestPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /** Adds vision data to the pose esimation. */
  public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    poseEstimator.addVisionData(visionData);
  }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(trackWidthX.get() / 2.0, trackWidthY.get() / 2.0),
      new Translation2d(trackWidthX.get() / 2.0, -trackWidthY.get() / 2.0),
      new Translation2d(-trackWidthX.get() / 2.0, trackWidthY.get() / 2.0),
      new Translation2d(-trackWidthX.get() / 2.0, -trackWidthY.get() / 2.0)
    };
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    isCharacterizing = true;
    characterizationVolts = volts;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }
}
