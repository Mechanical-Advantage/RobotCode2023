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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  public static final double trajectoryCacheMarginRadians = Units.degreesToRadians(3.0);
  public static final double shiftCenterMarginMeters = 0.05;
  public static final double wristGroundMarginMeters = 0.05;
  public static final double[] cubeIntakeAvoidanceRect = new double[] {0.05, 0.0, 0.75, 0.65};
  public static final double[] coneIntakeAvoidanceRect = new double[] {-0.65, 0.0, -0.05, 0.6};
  public static final double avoidanceLookaheadSecs = 0.25;
  public static final double emergencyDisableMaxError = Units.degreesToRadians(10.0);
  public static final double emergencyDisableMaxErrorTime = 0.5;
  public static final double emergencyDisableBeyondLimitThreshold = Units.degreesToRadians(5.0);

  private final ArmIO io;
  private final ArmSolverIO solverIo;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmSolverIOInputsAutoLogged solverInputs = new ArmSolverIOInputsAutoLogged();

  static final String configFilename = "arm_config.json";
  private final String configJson;
  private final ArmConfig config;
  private final ArmKinematics kinematics;
  private final ArmDynamics dynamics;

  private Supplier<Boolean> disableSupplier = () -> false;
  private Supplier<Boolean> forcePregeneratedSupplier = () -> false;
  private boolean emergencyDisable = false;
  private Timer emergencyDisableMaxErrorTimer = new Timer();
  private Alert driverDisableAlert =
      new Alert("Arm disabled due to driver override.", AlertType.WARNING);
  private Alert emergencyDisableAlert =
      new Alert(
          "Arm emergency disabled due to high position error. Disable the arm manually and reenable to reset.",
          AlertType.ERROR);

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
    ArmVisualizer.logRectangleConstraints("ArmConstraints", config, new Color8Bit(Color.kBlack));
    ArmVisualizer.logRectangles(
        "ArmIntakeAvoidance",
        new double[][] {cubeIntakeAvoidanceRect, coneIntakeAvoidanceRect},
        new Color8Bit(Color.kGreen));

    // Load cached trajectories
    for (var trajectory : ArmTrajectoryCache.loadTrajectories()) {
      allTrajectories.put(trajectory.getParameters().hashCode(), trajectory);
    }
  }

  public void setOverrides(
      Supplier<Boolean> disableSupplier, Supplier<Boolean> forcePregeneratedSupplier) {
    this.disableSupplier = disableSupplier;
    this.forcePregeneratedSupplier = forcePregeneratedSupplier;
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

  /** Returns whether the arm should be prevented from moving. */
  private boolean isDisabled() {
    return DriverStation.isDisabled() || disableSupplier.get() || emergencyDisable;
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
              - inputs.shoulderRelativePositionRad;
      elbowAngleOffset =
          MathUtil.inputModulus(inputs.elbowAbsolutePositionRad, 0.0, Math.PI * 2.0)
              - inputs.elbowRelativePositionRad;
      wristAngleOffset =
          MathUtil.inputModulus(inputs.wristAbsolutePositionRad, -Math.PI, Math.PI)
              - inputs.wristRelativePositionRad;
      isZeroed = true;
    }

    // Get measured positions
    shoulderAngle = inputs.shoulderRelativePositionRad + shoulderAngleOffset;
    elbowAngle = inputs.elbowRelativePositionRad + elbowAngleOffset;
    wristAngle = inputs.wristRelativePositionRad + wristAngleOffset;

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

    // Set setpoint to current position when disabled (don't move when enabling)
    if (isDisabled()) {
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

    // Values to be logged at the end of the cycle and used for emergency stop
    double shoulderAngleSetpoint = Math.PI / 2.0;
    double elbowAngleSetpoint = Math.PI;
    double wristAngleSetpoint = 0.0;

    // Run shoulder and elbow
    ArmPose wristEffectiveArmPose = null; // The arm pose to use for wrist calculations
    if (isDisabled()) {
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
      shoulderAngleSetpoint = state.get(0, 0);
      elbowAngleSetpoint = state.get(1, 0);

    } else {
      // Go to setpoint
      Optional<Vector<N2>> angles = kinematics.inverse(setpointPose.endEffectorPosition());
      if (angles.isPresent()) {
        var voltages = dynamics.feedforward(angles.get());
        io.setShoulderVoltage(
            voltages.get(0, 0) + shoulderFeedback.calculate(shoulderAngle, angles.get().get(0, 0)));
        io.setElbowVoltage(
            voltages.get(1, 0) + elbowFeedback.calculate(elbowAngle, angles.get().get(1, 0)));
        shoulderAngleSetpoint = angles.get().get(0, 0);
        elbowAngleSetpoint = angles.get().get(1, 0);
      } else {
        io.setShoulderVoltage(0.0);
        io.setElbowVoltage(0.0);
      }
      if (currentTrajectory == null) {
        wristEffectiveArmPose =
            setpointPose; // Move wrist based on current setpoint (no trajectory in queue)
      } else {
        wristEffectiveArmPose = queuedPose; // Move wrist based on queued trajectory
      }
    }

    // Run wrist
    if (isDisabled() || wristEffectiveArmPose == null) {
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
        wristAngleSetpoint = wristFeedback.getSetpoint().position;
      }
    }

    // Log setpoints and measured positions
    visualizerMeasured.update(shoulderAngle, elbowAngle, wristAngle);
    visualizerSetpoint.update(shoulderAngleSetpoint, elbowAngleSetpoint, wristAngleSetpoint);
    Logger.getInstance().recordOutput("Arm/MeasuredAngle/Shoulder", shoulderAngle);
    Logger.getInstance().recordOutput("Arm/MeasuredAngle/Elbow", elbowAngle);
    Logger.getInstance().recordOutput("Arm/MeasuredAngle/Wrist", wristAngle);
    Logger.getInstance().recordOutput("Arm/SetpointAngle/Shoulder", shoulderAngleSetpoint);
    Logger.getInstance().recordOutput("Arm/SetpointAngle/Elbow", elbowAngleSetpoint);
    Logger.getInstance().recordOutput("Arm/SetpointAngle/Wrist", wristAngleSetpoint);
    Logger.getInstance()
        .recordOutput("Arm/SetpointPose/X", setpointPose.endEffectorPosition().getX());
    Logger.getInstance()
        .recordOutput("Arm/SetpointPose/Y", setpointPose.endEffectorPosition().getY());
    Logger.getInstance()
        .recordOutput("Arm/SetpointPose/Wrist", setpointPose.globalWristAngle().getRadians());

    // Trigger emergency stop if necessary
    if (Constants.getMode() != Mode.SIM) {
      emergencyDisableMaxErrorTimer.start();
      if (isDisabled()) {
        emergencyDisableMaxErrorTimer.reset();
      } else {
        // Check for high error
        if (Math.abs(shoulderAngle - shoulderAngleSetpoint) < emergencyDisableMaxError
            && Math.abs(elbowAngle - elbowAngleSetpoint) < emergencyDisableMaxError
            && Math.abs(wristAngle - wristAngleSetpoint) < emergencyDisableMaxError) {
          emergencyDisableMaxErrorTimer.reset();
        } else if (emergencyDisableMaxErrorTimer.hasElapsed(emergencyDisableMaxErrorTime)) {
          emergencyDisable = true;
        }

        // Check if beyond limits
        if (shoulderAngle < config.shoulder().minAngle() - emergencyDisableBeyondLimitThreshold
            || shoulderAngle > config.shoulder().maxAngle() + emergencyDisableBeyondLimitThreshold
            || elbowAngle < config.elbow().minAngle() - emergencyDisableBeyondLimitThreshold
            || elbowAngle > config.elbow().maxAngle() + emergencyDisableBeyondLimitThreshold
            || wristAngle < config.wrist().minAngle() - emergencyDisableBeyondLimitThreshold
            || wristAngle > config.wrist().maxAngle() + emergencyDisableBeyondLimitThreshold) {
          emergencyDisable = true;
        }
      }
    }

    // Reset internal emergency stop when override is active
    if (disableSupplier.get()) {
      emergencyDisable = false;
    }

    // Update disable alerts
    driverDisableAlert.set(disableSupplier.get());
    emergencyDisableAlert.set(emergencyDisable);
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

  /** Returns the current arm setpoint. */
  public ArmPose getSetpoint() {
    return setpointPose != null ? setpointPose : ArmPose.Preset.HOMED.getPose();
  }

  /** Returns whether the current current is complete. */
  public boolean isTrajectoryFinished() {
    return currentTrajectory == null && wristFeedback.getGoal().equals(wristFeedback.getSetpoint());
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
      DriverStation.reportWarning("Infeasible arm trajectory requested", false);
      return;
    }

    // Exit if already at setpoint
    if (Math.abs(currentAngles.get().get(0, 0) - targetAngles.get().get(0, 0))
            <= trajectoryCacheMarginRadians
        && Math.abs(currentAngles.get().get(1, 0) - targetAngles.get().get(1, 0))
            <= trajectoryCacheMarginRadians) {
      currentTrajectory = null;
      setpointPose = pose;
      return;
    }

    // Reset current trajectory
    trajectoryTimer.stop();
    trajectoryTimer.reset();

    // Create trajectory and search for similar
    var parameters = new ArmTrajectory.Parameters(currentAngles.get(), targetAngles.get());
    var trajectory = new ArmTrajectory(parameters);
    ArmTrajectory closestTrajectory = trajectory.findClosest(allTrajectories.values());

    // If close enough or overridden, use this trajectory
    if (closestTrajectory != null) {
      var initialDiff =
          trajectory
              .getParameters()
              .initialJointPositions()
              .minus(closestTrajectory.getParameters().initialJointPositions());
      var finalDiff =
          trajectory
              .getParameters()
              .finalJointPositions()
              .minus(closestTrajectory.getParameters().finalJointPositions());
      if (forcePregeneratedSupplier.get()
          || (Math.abs(initialDiff.get(0, 0)) <= trajectoryCacheMarginRadians
              && Math.abs(initialDiff.get(1, 0)) <= trajectoryCacheMarginRadians
              && Math.abs(finalDiff.get(0, 0)) <= trajectoryCacheMarginRadians
              && Math.abs(finalDiff.get(1, 0)) <= trajectoryCacheMarginRadians
              && closestTrajectory.isGenerated())) {
        currentTrajectory = closestTrajectory;
        queuedPose = pose;
        return;
      }
    }

    // Start new trajectory
    allTrajectories.put(parameters.hashCode(), trajectory);
    currentTrajectory = trajectory;
    queuedPose = pose;
    updateTrajectoryRequest(); // Start solving immediately if nothing else in queue

    // Pregenerate return to homed if not cached
    Vector<N2> homedAngles =
        kinematics.inverse(ArmPose.Preset.HOMED.getPose().endEffectorPosition()).get();
    var diff = targetAngles.get().minus(homedAngles);
    if (Math.abs(diff.get(0, 0)) > trajectoryCacheMarginRadians
        || Math.abs(diff.get(1, 0)) > trajectoryCacheMarginRadians) {
      var returnTrajectory =
          new ArmTrajectory(new ArmTrajectory.Parameters(targetAngles.get(), homedAngles));
      ArmTrajectory closestReturnTrajectory =
          returnTrajectory.findClosest(allTrajectories.values());
      var initialDiff =
          returnTrajectory
              .getParameters()
              .initialJointPositions()
              .minus(closestReturnTrajectory.getParameters().initialJointPositions());
      var finalDiff =
          returnTrajectory
              .getParameters()
              .finalJointPositions()
              .minus(closestReturnTrajectory.getParameters().finalJointPositions());
      if (Math.abs(initialDiff.get(0, 0)) > trajectoryCacheMarginRadians
          || Math.abs(initialDiff.get(1, 0)) > trajectoryCacheMarginRadians
          || Math.abs(finalDiff.get(0, 0)) > trajectoryCacheMarginRadians
          || Math.abs(finalDiff.get(1, 0)) > trajectoryCacheMarginRadians) {
        allTrajectories.put(returnTrajectory.getParameters().hashCode(), returnTrajectory);
      }
    }
  }

  /** Command factory to navigate to a pose along a path. */
  public Command runPathCommand(ArmPose.Preset preset) {
    return runPathCommand(preset.getPose());
  }

  /** Command factory to navigate to a pose along a path. */
  public Command runPathCommand(ArmPose pose) {
    return runPathCommand(() -> pose);
  }

  /** Command factory to navigate to a pose along a path. */
  public Command runPathCommand(Supplier<ArmPose> pose) {
    return runOnce(() -> runPath(pose.get()))
        .andThen(Commands.waitUntil(this::isTrajectoryFinished));
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

  /**
   * Returns the maximum reach (x coordinate relative to the arm origin) that the arm can achieve at
   * the provided height.
   */
  public double calcMaxReachAtHeight(double height) {
    return kinematics.calcMaxReachAtHeight(height);
  }
}
