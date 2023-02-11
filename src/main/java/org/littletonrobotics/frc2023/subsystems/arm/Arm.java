// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.commands.DriveToNode;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  static final double trajectoryCacheMarginRadians = 0.02;
  static final double shiftCenterMarginMeters = 0.05;
  static final double wristGroundMarginMeters = 0.05;
  static final double[] cubeIntakeAvoidanceRect = new double[] {0.01, 0.0, 0.8, 0.6};
  static final double[] coneIntakeAvoidanceRect = new double[] {-0.6, 0.0, -0.01, 0.6};
  static final double avoidanceLookaheadSecs = 0.25;

  private final ArmIO io;
  private final ArmSolverIO solverIo;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmSolverIOInputsAutoLogged solverInputs = new ArmSolverIOInputsAutoLogged();

  static final String configFilename = "arm_config.json";
  private final String configJson;
  private final ArmConfig config;
  private final ArmKinematics kinematics;
  private final ArmDynamics dynamics;

  private final ArmVisualizer visualizerMeasured;
  private final ArmVisualizer visualizerSetpoint;

  private boolean isZeroed = false;
  private double shoulderAngleOffset = 0.0;
  private double elbowAngleOffset = 0.0;
  private double wristAngleOffset = 0.0;
  private double shoulderAngle = 0.0;
  private double elbowAngle = 0.0;
  private double wristAngle = 0.0;

  private final Map<Integer, ArmTrajectory> allTrajectories = new HashMap<>();
  private ArmTrajectory currentTrajectory = null;
  private ArmPose setpointPose = null; // Pose to revert to when not following trajectory
  private ArmPose queuedPose = null; // Use as setpoint once trajectory is completed
  private Timer trajectoryTimer = new Timer();
  private boolean presetMessagePrinted = false;
  private int presetTrajectoryCount = 0;

  private PIDController shoulderFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private PIDController elbowFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private ProfiledPIDController wristFeedback =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private static final LoggedTunableNumber shoulderKp = new LoggedTunableNumber("Arm/Shoulder/kP");
  private static final LoggedTunableNumber shoulderKd = new LoggedTunableNumber("Arm/Shoulder/kD");
  private static final LoggedTunableNumber elbowKp = new LoggedTunableNumber("Arm/Elbow/kP");
  private static final LoggedTunableNumber elbowKd = new LoggedTunableNumber("Arm/Elbow/kD");
  private static final LoggedTunableNumber wristKp = new LoggedTunableNumber("Arm/Wrist/kP");
  private static final LoggedTunableNumber wristKd = new LoggedTunableNumber("Arm/Wrist/kD");
  private static final LoggedTunableNumber wristMaxVelocity =
      new LoggedTunableNumber("Arm/Wrist/MaxVelocity");
  private static final LoggedTunableNumber wristMaxAcceleration =
      new LoggedTunableNumber("Arm/Wrist/MaxAcceleration");

  static {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        shoulderKp.initDefault(80.0);
        shoulderKd.initDefault(0.0);
        elbowKp.initDefault(80.0);
        elbowKd.initDefault(0.0);
        wristKp.initDefault(80.0);
        wristKd.initDefault(2.0);
        wristMaxVelocity.initDefault(10.0);
        wristMaxAcceleration.initDefault(50.0);
        break;
      default:
        break;
    }
  }

  /** Represents a target position for the arm. */
  public static record ArmPose(Translation2d endEffectorPosition, Rotation2d globalWristAngle) {
    public static enum Preset {
      HOMED(null),
      SINGLE_SUBTATION(
          new ArmPose(
              new Translation2d(0.6, FieldConstants.LoadingZone.singleSubstationCenterZ),
              new Rotation2d())),
      DOUBLE_SUBTATION(
          new ArmPose(
              new Translation2d(0.5, FieldConstants.LoadingZone.doubleSubstationShelfZ + 0.1),
              new Rotation2d())),
      SCORE_HYBRID(new ArmPose(new Translation2d(0.55, 0.9), Rotation2d.fromDegrees(-90.0))),
      SCORE_MID_CONE(
          new ArmPose(
              new Translation2d(
                  DriveToNode.scorePositionX - FieldConstants.Grids.midX - 0.2,
                  FieldConstants.Grids.midConeZ + 0.2),
              Rotation2d.fromDegrees(30.0))),
      SCORE_MID_CUBE(
          new ArmPose(
              new Translation2d(
                  DriveToNode.scorePositionX - FieldConstants.Grids.midX - 0.3,
                  FieldConstants.Grids.midCubeZ + 0.5),
              Rotation2d.fromDegrees(-45.0))),
      SCORE_HIGH_CONE(
          new ArmPose(
              new Translation2d(
                  DriveToNode.scorePositionX - FieldConstants.Grids.highX - 0.2,
                  FieldConstants.Grids.highConeZ + 0.2),
              Rotation2d.fromDegrees(30.0))),
      SCORE_HIGH_CUBE(
          new ArmPose(
              new Translation2d(
                  DriveToNode.scorePositionX - FieldConstants.Grids.highX - 0.3,
                  FieldConstants.Grids.highCubeZ + 0.5),
              Rotation2d.fromDegrees(-45.0)));

      private ArmPose pose;

      private Preset(ArmPose pose) {
        this.pose = pose;
      }

      private void setPose(ArmPose pose) {
        this.pose = pose;
      }

      public ArmPose getPose() {
        return pose;
      }

      public static void updateHomedPreset(ArmConfig config) {
        HOMED.setPose(
            new ArmPose(
                new Translation2d(
                    config.origin().getX(),
                    config.origin().getY() + config.shoulder().length() - config.elbow().length()),
                new Rotation2d(-Math.PI / 2)));
      }
    }

    private static double wristLength = 0.0;

    public Translation2d wristPosition() {
      return new Pose2d(endEffectorPosition, globalWristAngle)
          .transformBy(new Transform2d(new Translation2d(wristLength, 0.0), new Rotation2d()))
          .getTranslation();
    }

    public ArmPose withFlip(boolean flip) {
      return flip
          ? new ArmPose(
              new Translation2d(-this.endEffectorPosition.getX(), this.endEffectorPosition.getY()),
              new Rotation2d(-this.globalWristAngle.getCos(), this.globalWristAngle.getSin()))
          : this;
    }
  }

  public Arm(ArmIO io, ArmSolverIO solverIo) {
    this.io = io;
    this.solverIo = solverIo;
    io.setBrakeMode(true, true, true);

    // Get config from JSON
    File configFile = new File(Filesystem.getDeployDirectory(), configFilename);
    try {
      configJson = Files.readString(configFile.toPath());
    } catch (IOException e) {
      throw new RuntimeException("Failed to read raw arm config JSON");
    }
    config = ArmConfig.loadJson(configFile);
    io.setConfig(config);
    solverIo.setConfig(configJson);
    kinematics = new ArmKinematics(config);
    dynamics = new ArmDynamics(config);
    ArmPose.wristLength = config.wrist().length();
    ArmPose.Preset.updateHomedPreset(config);

    // Create visualizers
    visualizerMeasured = new ArmVisualizer(config, "ArmMeasured", null);
    visualizerSetpoint = new ArmVisualizer(config, "ArmSetpoint", new Color8Bit(Color.kOrange));
    ArmVisualizer.logRectConstraints(config);

    // Load cached trajectories
    for (var trajectory : ArmTrajectoryCache.loadTrajectories()) {
      allTrajectories.put(trajectory.getParameters().hashCode(), trajectory);
    }
  }

  /**
   * Finds the next ungenerated trajectory and request it from the solver. If that trajectory is
   * already being generated this will have no effect.
   */
  private void updateTrajectoryRequest() {
    for (var trajectory : allTrajectories.values()) {
      if (!trajectory.isGenerated()) {
        solverIo.request(trajectory.getParameters());
        break;
      }
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    solverIo.updateInputs(solverInputs);
    Logger.getInstance().processInputs("Arm", inputs);
    Logger.getInstance().processInputs("ArmSolver", solverInputs);

    // Update tunable numbers
    if (shoulderKp.hasChanged(hashCode()) || shoulderKd.hasChanged(hashCode())) {
      shoulderFeedback.setP(shoulderKp.get());
      shoulderFeedback.setD(shoulderKd.get());
    }
    if (elbowKp.hasChanged(hashCode()) || elbowKd.hasChanged(hashCode())) {
      elbowFeedback.setP(elbowKp.get());
      elbowFeedback.setD(elbowKd.get());
    }
    if (wristKp.hasChanged(hashCode()) || wristKd.hasChanged(hashCode())) {
      wristFeedback.setP(wristKp.get());
      wristFeedback.setD(wristKd.get());
    }
    if (wristMaxVelocity.hasChanged(hashCode()) && wristMaxAcceleration.hasChanged(hashCode())) {
      wristFeedback.setConstraints(
          new TrapezoidProfile.Constraints(wristMaxVelocity.get(), wristMaxAcceleration.get()));
    }

    // Zero with absolute encoders
    if (!isZeroed) {
      shoulderAngleOffset =
          MathUtil.inputModulus(inputs.shoulderAbsolutePositionRad, -Math.PI, Math.PI)
              - inputs.shoulderPositionRad;
      elbowAngleOffset =
          MathUtil.inputModulus(inputs.elbowAbsolutePositionRad, 0.0, Math.PI * 2.0)
              - inputs.elbowPositionRad;
      wristAngleOffset =
          MathUtil.inputModulus(inputs.wristAbsolutePositionRad, -Math.PI, Math.PI)
              - inputs.wristPositionRad;
      isZeroed = true;
    }

    // Get measured positions
    shoulderAngle = inputs.shoulderPositionRad + shoulderAngleOffset;
    elbowAngle = inputs.elbowPositionRad + elbowAngleOffset;
    wristAngle = inputs.wristPositionRad + wristAngleOffset;
    visualizerMeasured.update(shoulderAngle, elbowAngle, wristAngle);

    // Get new trajectory from solver if available
    if (solverInputs.parameterHash != 0
        && allTrajectories.containsKey((int) solverInputs.parameterHash)) {
      var trajectory = allTrajectories.get((int) solverInputs.parameterHash);
      if (!trajectory.isGenerated()) {
        List<Vector<N2>> points = new ArrayList<>();
        for (int i = 0; i < solverInputs.shoulderPoints.length; i++) {
          points.add(VecBuilder.fill(solverInputs.shoulderPoints[i], solverInputs.elbowPoints[i]));
        }
        trajectory.setPoints(solverInputs.totalTime, points);
      }
    }

    // Request next trajectory from solver
    updateTrajectoryRequest();

    // Log status of cached trajectories
    int trajectoryCount = allTrajectories.size();
    int trajectoryCountGenerated = 0;
    for (var trajectory : allTrajectories.values()) {
      if (trajectory.isGenerated()) trajectoryCountGenerated++;
    }
    Logger.getInstance().recordOutput("Arm/TrajectoryCount", trajectoryCount);
    Logger.getInstance().recordOutput("Arm/TrajectoryCountGenerated", trajectoryCountGenerated);
    if (trajectoryCountGenerated >= presetTrajectoryCount) {
      if (!presetMessagePrinted) {
        System.out.println("All preset arm trajectories ready!");
        presetMessagePrinted = true;
      }
    }

    // Set setpoint to current position when disabled (don't move when enabling)
    if (DriverStation.isDisabled()) {
      setpointPose =
          new ArmPose(
              kinematics.forward(VecBuilder.fill(shoulderAngle, elbowAngle)),
              new Rotation2d(shoulderAngle + elbowAngle + wristAngle));
      currentTrajectory = null;
      queuedPose = null;
    }

    // Check if trajectory is finished
    if (currentTrajectory != null
        && currentTrajectory.isGenerated()
        && trajectoryTimer.hasElapsed(currentTrajectory.getTotalTime())) {
      trajectoryTimer.stop();
      trajectoryTimer.reset();
      currentTrajectory = null;
      setpointPose = queuedPose;
    }

    // Values to be logged at the end of the cycle
    double loggedShoulderSetpoint = Math.PI / 2.0;
    double loggedElbowSetpoint = Math.PI;
    double loggedWristSetpoint = 0.0;

    // Run shoulder and elbow
    ArmPose wristEffectiveArmPose = null; // The arm pose to use for wrist calculations
    if (DriverStation.isDisabled()) {
      // Stop moving when disabled
      io.setShoulderVoltage(0.0);
      io.setElbowVoltage(0.0);
      shoulderFeedback.reset();
      elbowFeedback.reset();

    } else if (currentTrajectory != null && currentTrajectory.isGenerated()) {
      // Follow trajectory
      trajectoryTimer.start();
      var state = currentTrajectory.sample(trajectoryTimer.get());
      var voltages = dynamics.feedforward(state);
      io.setShoulderVoltage(
          voltages.get(0, 0) + shoulderFeedback.calculate(shoulderAngle, state.get(0, 0)));
      io.setElbowVoltage(voltages.get(1, 0) + elbowFeedback.calculate(elbowAngle, state.get(1, 0)));
      setpointPose = // If trajectory is interrupted, go to last setpoint
          new ArmPose(
              kinematics.forward(new Vector<>(state.extractColumnVector(0))),
              queuedPose.globalWristAngle());
      wristEffectiveArmPose = queuedPose; // Move wrist based on trajectory endpoint
      loggedShoulderSetpoint = state.get(0, 0);
      loggedElbowSetpoint = state.get(1, 0);

    } else {
      // Go to setpoint
      Optional<Vector<N2>> angles = kinematics.inverse(setpointPose.endEffectorPosition());
      if (angles.isPresent()) {
        var voltages = dynamics.feedforward(angles.get());
        io.setShoulderVoltage(
            voltages.get(0, 0) + shoulderFeedback.calculate(shoulderAngle, angles.get().get(0, 0)));
        io.setElbowVoltage(
            voltages.get(1, 0) + elbowFeedback.calculate(elbowAngle, angles.get().get(1, 0)));
        loggedShoulderSetpoint = angles.get().get(0, 0);
        loggedElbowSetpoint = angles.get().get(1, 0);
      } else {
        io.setShoulderVoltage(0.0);
        io.setElbowVoltage(0.0);
      }
      wristEffectiveArmPose = setpointPose; // Move wrist based on current setpoint
    }

    // Run wrist
    if (DriverStation.isDisabled() || wristEffectiveArmPose == null) {
      // Stop moving when disabled
      io.setWristVoltage(0.0);
      wristFeedback.reset(wristAngle);

    } else {
      // Go to setpoint
      Optional<Vector<N2>> shoulderElbowAngles =
          kinematics.inverse(wristEffectiveArmPose.endEffectorPosition());
      if (shoulderElbowAngles.isPresent()) {
        // Calculate required angle to avoid hitting the ground
        double globalAngleRadians =
            MathUtil.angleModulus(wristEffectiveArmPose.globalWristAngle().getRadians());
        double groundDistance =
            wristEffectiveArmPose.endEffectorPosition().getY() - wristGroundMarginMeters;

        if (groundDistance < config.wrist().length()) {
          double minGroundAngle = Math.acos(groundDistance / config.wrist().length());
          if (globalAngleRadians >= -Math.PI / 2
              && globalAngleRadians < -Math.PI / 2 + minGroundAngle) {
            globalAngleRadians = -Math.PI / 2 + minGroundAngle;
          }
          if (globalAngleRadians < -Math.PI / 2
              && globalAngleRadians > -Math.PI / 2 - minGroundAngle) {
            globalAngleRadians = -Math.PI / 2 - minGroundAngle;
          }
        }

        // Calculate setpoint and run controller
        double wristSetpoint =
            new Rotation2d(globalAngleRadians)
                .minus(
                    new Rotation2d(
                        shoulderElbowAngles.get().get(0, 0) + shoulderElbowAngles.get().get(1, 0)))
                .getRadians();
        wristSetpoint = MathUtil.angleModulus(wristSetpoint);
        wristSetpoint =
            MathUtil.clamp(wristSetpoint, config.wrist().minAngle(), config.wrist().maxAngle());
        io.setWristVoltage(wristFeedback.calculate(wristAngle, wristSetpoint));
        loggedWristSetpoint = wristFeedback.getSetpoint().position;
      }
    }

    // Log setpoints
    visualizerSetpoint.update(loggedShoulderSetpoint, loggedElbowSetpoint, loggedWristSetpoint);
    Logger.getInstance().recordOutput("Arm/SetpointX", setpointPose.endEffectorPosition().getX());
    Logger.getInstance().recordOutput("Arm/SetpointY", setpointPose.endEffectorPosition().getY());
    Logger.getInstance()
        .recordOutput("Arm/SetpointWrist", setpointPose.globalWristAngle().getRadians());
  }

  /** Returns whether the arm is current in or will pass through the provided region. */
  private boolean checkAvoidanceRegion(double[] region) {
    // Check setpoint
    if (setpointPose.endEffectorPosition().getX() >= region[0]
        && setpointPose.endEffectorPosition().getX() <= region[2]
        && setpointPose.endEffectorPosition().getY() >= region[1]
        && setpointPose.endEffectorPosition().getY() <= region[3]) {
      return true;
    }
    var setpointWristPosition = setpointPose.wristPosition();
    if (setpointWristPosition.getX() >= region[0]
        && setpointWristPosition.getX() <= region[2]
        && setpointWristPosition.getY() >= region[1]
        && setpointWristPosition.getY() <= region[3]) {
      return true;
    }

    // Check trajectory
    if (currentTrajectory != null && currentTrajectory.isGenerated()) {
      var points = currentTrajectory.getPoints();
      var dt = currentTrajectory.getTotalTime() / (points.size() - 1);
      for (int i = 0; i < points.size(); i++) {
        var time = dt * i;
        if (time < trajectoryTimer.get()) {
          continue;
        }
        if (time > trajectoryTimer.get() + avoidanceLookaheadSecs) {
          break;
        }
        var endEffectorPosition = kinematics.forward(points.get(i));
        var wristPosition =
            new ArmPose(endEffectorPosition, queuedPose.globalWristAngle()).wristPosition();
        if (endEffectorPosition.getX() >= region[0]
            && endEffectorPosition.getX() <= region[2]
            && endEffectorPosition.getY() >= region[1]
            && endEffectorPosition.getY() <= region[3]) {
          return true;
        }
        if (wristPosition.getX() >= region[0]
            && wristPosition.getX() <= region[2]
            && wristPosition.getY() >= region[1]
            && wristPosition.getY() <= region[3]) {
          return true;
        }
      }
    }

    return false;
  }

  /** Returns whether the cube intake should be extended to avoid colliding with the arm. */
  public boolean cubeIntakeShouldExtend() {
    return checkAvoidanceRegion(cubeIntakeAvoidanceRect);
  }

  /** Returns whether the cone intake should be extended to avoid colliding with the arm. */
  public boolean coneIntakeShouldExtend() {
    return checkAvoidanceRegion(coneIntakeAvoidanceRect);
  }

  /** Returns whether the current current is complete. */
  public boolean isTrajectoryFinished() {
    return currentTrajectory == null;
  }

  /** Starts navigating to a pose. */
  public void runPath(ArmPose.Preset preset) {
    runPath(preset.getPose());
  }

  /** Starts navigating to a pose. */
  public void runPath(ArmPose pose) {
    // Get current and target angles
    Optional<Vector<N2>> currentAngles = kinematics.inverse(setpointPose.endEffectorPosition());
    Optional<Vector<N2>> targetAngles = kinematics.inverse(pose.endEffectorPosition());
    if (currentAngles.isEmpty() || targetAngles.isEmpty()) {
      return;
    }

    // Exit if already at setpoint
    if (Math.abs(currentAngles.get().get(0, 0) - targetAngles.get().get(0, 0))
            < trajectoryCacheMarginRadians
        && Math.abs(currentAngles.get().get(1, 0) - targetAngles.get().get(1, 0))
            < trajectoryCacheMarginRadians) {
      currentTrajectory = null;
      setpointPose = pose;
      return;
    }

    // Create parameters
    var parameters = new ArmTrajectory.Parameters(currentAngles.get(), targetAngles.get());

    // Reset current trajectory
    trajectoryTimer.stop();
    trajectoryTimer.reset();

    // Search for similar trajectories
    for (var trajectory : allTrajectories.values()) {
      var initialDiff =
          trajectory
              .getParameters()
              .initialJointPositions()
              .minus(parameters.initialJointPositions());
      var finalDiff =
          trajectory.getParameters().finalJointPositions().minus(parameters.finalJointPositions());
      if (Math.abs(initialDiff.get(0, 0)) < trajectoryCacheMarginRadians
          && Math.abs(initialDiff.get(1, 0)) < trajectoryCacheMarginRadians
          && Math.abs(finalDiff.get(0, 0)) < trajectoryCacheMarginRadians
          && Math.abs(finalDiff.get(1, 0)) < trajectoryCacheMarginRadians
          && trajectory.isGenerated()) { // If not generated, try again
        currentTrajectory = trajectory;
        queuedPose = pose;
        return;
      }
    }

    // Create new trajectory
    var trajectory = new ArmTrajectory(parameters);
    allTrajectories.put(parameters.hashCode(), trajectory);
    currentTrajectory = trajectory;
    queuedPose = pose;
    updateTrajectoryRequest(); // Start solving immediately if nothing else in queue
  }

  /** Command factory to navigate to a pose along a path. */
  public Command runPathCommand(ArmPose.Preset preset) {
    return runPathCommand(preset.getPose());
  }

  /** Command factory to navigate to a pose along a path. */
  public Command runPathCommand(ArmPose pose) {
    return runOnce(() -> runPath(pose)).andThen(Commands.waitUntil(this::isTrajectoryFinished));
  }

  /** Go directly to the provided pose without using a path. Only use for small movements. */
  public void runDirect(ArmPose pose) {
    var translation = pose.endEffectorPosition().minus(config.origin());

    // Keep translation within arm inner and outer ranges
    double innerRadius = Math.abs(config.shoulder().length() - config.elbow().length());
    double outerRadius = config.shoulder().length() + config.elbow().length();
    if (translation.getNorm() < innerRadius) {
      translation = translation.times(innerRadius / translation.getNorm());
    } else if (translation.getNorm() > outerRadius) {
      translation = translation.times(outerRadius / translation.getNorm());
    }

    // Keep translation on same side of root
    double setpointX = setpointPose.endEffectorPosition().minus(config.origin()).getX();
    if (setpointX > 1e-3) {
      if (translation.getX() < shiftCenterMarginMeters) {
        translation = new Translation2d(shiftCenterMarginMeters, translation.getY());
      }
    } else if (setpointX < -1e-3) {
      if (translation.getX() > -shiftCenterMarginMeters) {
        translation = new Translation2d(-shiftCenterMarginMeters, translation.getY());
      }
    }

    // Don't go into the ground
    translation = translation.plus(config.origin());
    if (translation.getY() < wristGroundMarginMeters) {
      translation = new Translation2d(translation.getX(), wristGroundMarginMeters);
    }

    // Apply new translation if valid
    var angles = kinematics.inverse(translation);
    if (angles.isPresent()) {
      currentTrajectory = null;
      setpointPose = new ArmPose(translation, pose.globalWristAngle());
    }
  }
}
