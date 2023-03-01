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
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.subsystems.leds.Leds;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  public static final double maxVoltageNoTrajectory =
      6.0; // Limit shoulder & elbow voltage when not following trajectory
  public static final double trajectoryCacheMarginRadians = Units.degreesToRadians(3.0);
  public static final double shiftCenterMarginMeters = 0.05;
  public static final double wristGroundMarginMeters = 0.05;
  public static final double[] cubeIntakeAvoidanceRect = new double[] {0.1, 0.0, 0.75, 0.65};
  public static final double nodeConstraintMinY =
      0.8; // If target or start is above this y, enable node constraints
  public static final double nodeConstraintMinX =
      0.52; // If target or start is beyond this x, enable node constraints
  public static final Set<String> frontNodeConstraints = Set.of("nodeMidFront", "nodeHighFront");
  public static final Set<String> backNodeConstraints = Set.of("nodeMidBack", "nodeHighBack");
  public static final double avoidanceLookaheadSecs = 0.5;
  public static final double emergencyDisableMaxError = Units.degreesToRadians(20.0);
  public static final double emergencyDisableMaxErrorTime = 1.0;
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
  private Supplier<Boolean> coastSupplier = () -> false;
  private Supplier<Boolean> forcePregeneratedSupplier = () -> false;
  private boolean lastCoast = false;
  private boolean emergencyDisable = false;
  private final Timer emergencyDisableMaxErrorTimer = new Timer();
  private final Alert driverDisableAlert =
      new Alert("Arm disabled due to driver override.", AlertType.WARNING);
  private final Alert emergencyDisableAlert =
      new Alert(
          "Arm emergency disabled due to high position error. Disable the arm manually and reenable to reset.",
          AlertType.ERROR);

  private final ArmVisualizer visualizerMeasured;
  private final ArmVisualizer visualizerSetpoint;

  private boolean isZeroed = false;
  private final Alert notZeroedAlert =
      new Alert(
          "Arm not zeroed due to ambiguous position, movement disabled. Please reposition the arm.",
          AlertType.ERROR);
  private double shoulderAngleOffset = 0.0;
  private double elbowAngleOffset = 0.0;
  private double wristAngleOffset = 0.0;
  private double shoulderAngle = 0.0;
  private double elbowAngle = 0.0;
  private double wristAngle = 0.0;
  private double extensionPercent = 0.0;

  private final Map<Integer, ArmTrajectory> allTrajectories = new HashMap<>();
  private ArmTrajectory currentTrajectory = null;
  private ArmPose setpointPose = null; // Pose to revert to when not following trajectory
  private ArmPose lastSetpointPose = null; // Used for FF calculation when not following trajectory
  private ArmPose queuedPose = null; // Use as setpoint once trajectory is completed
  private final Timer trajectoryTimer = new Timer();
  private double trajectoryStartWait = 0.0;

  private final PIDController shoulderFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final PIDController elbowFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final ProfiledPIDController wristFeedback =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private static final LoggedTunableNumber shoulderKp = new LoggedTunableNumber("Arm/Shoulder/kP");
  private static final LoggedTunableNumber shoulderKd = new LoggedTunableNumber("Arm/Shoulder/kD");
  private static final LoggedTunableNumber shoulderKs = new LoggedTunableNumber("Arm/Shoulder/kS");
  private static final LoggedTunableNumber shoulderKsDeadband =
      new LoggedTunableNumber("Arm/Shoulder/kSDeadband");
  private static final LoggedTunableNumber elbowKp = new LoggedTunableNumber("Arm/Elbow/kP");
  private static final LoggedTunableNumber elbowKd = new LoggedTunableNumber("Arm/Elbow/kD");
  private static final LoggedTunableNumber elbowKs = new LoggedTunableNumber("Arm/Elbow/kS");
  private static final LoggedTunableNumber elbowKsDeadband =
      new LoggedTunableNumber("Arm/Elbow/kSDeadband");
  private static final LoggedTunableNumber wristKp = new LoggedTunableNumber("Arm/Wrist/kP");
  private static final LoggedTunableNumber wristKd = new LoggedTunableNumber("Arm/Wrist/kD");
  private static final LoggedTunableNumber wristMaxVelocity =
      new LoggedTunableNumber("Arm/Wrist/MaxVelocity");
  private static final LoggedTunableNumber wristMaxAcceleration =
      new LoggedTunableNumber("Arm/Wrist/MaxAcceleration");

  static {
    if (!Constants.disableHAL) { // Don't run during trajectory cache generation
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
          shoulderKp.initDefault(6.0);
          shoulderKd.initDefault(0.2);
          shoulderKs.initDefault(0.1);
          shoulderKsDeadband.initDefault(0.05);
          elbowKp.initDefault(8.0);
          elbowKd.initDefault(0.4);
          elbowKs.initDefault(0.1);
          elbowKsDeadband.initDefault(0.05);
          wristKp.initDefault(20.0);
          wristKd.initDefault(0.0);
          wristMaxVelocity.initDefault(8.0);
          wristMaxAcceleration.initDefault(25.0);
          break;
        case ROBOT_SIMBOT:
          shoulderKp.initDefault(80.0);
          shoulderKd.initDefault(0.0);
          elbowKp.initDefault(80.0);
          elbowKd.initDefault(0.0);
          wristKp.initDefault(20.0);
          wristKd.initDefault(0.0);
          wristMaxVelocity.initDefault(8.0);
          wristMaxAcceleration.initDefault(25.0);
          break;
        default:
          break;
      }
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
        new double[][] {cubeIntakeAvoidanceRect},
        new Color8Bit(Color.kGreen));

    // Load cached trajectories
    for (var trajectory : ArmTrajectoryCache.loadTrajectories()) {
      allTrajectories.put(trajectory.getParameters().hashCode(), trajectory);
    }
  }

  public void setOverrides(
      Supplier<Boolean> disableSupplier,
      Supplier<Boolean> coastSupplier,
      Supplier<Boolean> forcePregeneratedSupplier) {
    this.disableSupplier = disableSupplier;
    this.coastSupplier = coastSupplier;
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
    return DriverStation.isDisabled() || disableSupplier.get() || emergencyDisable || !isZeroed;
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
    if (wristMaxVelocity.hasChanged(hashCode()) || wristMaxAcceleration.hasChanged(hashCode())) {
      wristFeedback.setConstraints(
          new TrapezoidProfile.Constraints(wristMaxVelocity.get(), wristMaxAcceleration.get()));
    }

    // Update coast mode
    boolean coast = coastSupplier.get() && DriverStation.isDisabled();
    if (coast != lastCoast) {
      lastCoast = coast;
      io.setBrakeMode(!coast, !coast, !coast);
    }
    Leds.getInstance().armCoast = coast;

    // Zero with absolute encoders
    if (!isZeroed) {
      // Check if elbow is above horizontal (ambiguous, maybe we're past the limits)
      double elbowAngle =
          MathUtil.inputModulus(inputs.elbowAbsolutePositionRad, 0.0, Math.PI * 2.0);
      if (elbowAngle > Math.PI / 2.0 && elbowAngle < 3.0 * Math.PI / 2.0) {
        // Elbow angle is below horizontal, zero normally
        shoulderAngleOffset =
            MathUtil.inputModulus(inputs.shoulderAbsolutePositionRad, -Math.PI, Math.PI)
                - inputs.shoulderRelativePositionRad;
        elbowAngleOffset = elbowAngle - inputs.elbowRelativePositionRad;
        wristAngleOffset =
            MathUtil.inputModulus(inputs.wristAbsolutePositionRad, -Math.PI, Math.PI)
                - inputs.wristRelativePositionRad;
        isZeroed = true;
      }
    }
    notZeroedAlert.set(!isZeroed);

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
      lastSetpointPose = setpointPose; // No velocity
      currentTrajectory = null;
      queuedPose = null;
    }

    // Stop trajectory if finished
    if (currentTrajectory != null
        && currentTrajectory.isGenerated()
        && trajectoryTimer.hasElapsed(trajectoryStartWait + currentTrajectory.getTotalTime())) {
      trajectoryTimer.stop();
      trajectoryTimer.reset();
      currentTrajectory = null;
      setpointPose = queuedPose;
    }

    // Start trajectory if ready
    if (currentTrajectory != null
        && currentTrajectory.isGenerated()
        && trajectoryTimer.get() == 0.0) {
      trajectoryTimer.start();
      trajectoryStartWait = 0.0;
      double timeUntilCollision = getTimeUntilCollision(cubeIntakeAvoidanceRect);
      if (timeUntilCollision < avoidanceLookaheadSecs && timeUntilCollision != 0.0) {
        trajectoryStartWait = avoidanceLookaheadSecs - timeUntilCollision;
      }
    }

    // Values to be logged at the end of the cycle and used for emergency stop
    double shoulderAngleSetpoint = Math.PI / 2.0;
    double elbowAngleSetpoint = Math.PI;
    double wristAngleSetpoint = 0.0;
    double shoulderVoltageFeedforward = 0.0;
    double elbowVoltageFeedforward = 0.0;
    double shoulderVoltageFeedback = 0.0;
    double elbowVoltageFeedback = 0.0;
    double wristVoltageFeedback = 0.0;

    // Run shoulder and elbow
    ArmPose wristEffectiveArmPose = null; // The arm pose to use for wrist calculations
    if (isDisabled()) {
      // Stop moving when disabled
      io.setShoulderVoltage(0.0);
      io.setElbowVoltage(0.0);
      shoulderFeedback.reset();
      elbowFeedback.reset();

    } else if (currentTrajectory != null
        && currentTrajectory.isGenerated()
        && trajectoryTimer.get() > trajectoryStartWait) {
      // Follow trajectory
      var state = currentTrajectory.sample(trajectoryTimer.get() - trajectoryStartWait);
      shoulderAngleSetpoint = state.get(0, 0);
      elbowAngleSetpoint = state.get(1, 0);

      var voltages = dynamics.feedforward(state);
      shoulderVoltageFeedforward = voltages.get(0, 0);
      elbowVoltageFeedforward = voltages.get(1, 0);
      shoulderVoltageFeedback = shoulderFeedback.calculate(shoulderAngle, state.get(0, 0));
      elbowVoltageFeedback = elbowFeedback.calculate(elbowAngle, state.get(1, 0));
      io.setShoulderVoltage(
          shoulderVoltageFeedforward
              + applyKs(shoulderVoltageFeedback, shoulderKs.get(), shoulderKsDeadband.get()));
      io.setElbowVoltage(
          elbowVoltageFeedforward
              + applyKs(elbowVoltageFeedback, elbowKs.get(), elbowKsDeadband.get()));

      setpointPose = // If trajectory is interrupted, go to last setpoint
          new ArmPose(
              kinematics.forward(new Vector<>(state.extractColumnVector(0))),
              new Rotation2d(
                  new Vector<>(state.extractColumnVector(0)).elementSum()
                      + wristAngle) // Hold current wrist angle
              );
      wristEffectiveArmPose = queuedPose; // Move wrist based on trajectory endpoint

    } else {
      // Go to setpoint
      Optional<Vector<N2>> angles = kinematics.inverse(setpointPose.endEffectorPosition());
      Optional<Vector<N2>> lastAngles = kinematics.inverse(lastSetpointPose.endEffectorPosition());
      if (angles.isPresent() && lastAngles.isPresent()) {
        shoulderAngleSetpoint = angles.get().get(0, 0);
        elbowAngleSetpoint = angles.get().get(1, 0);
        Vector<N2> velocities =
            new Vector<>(angles.get().minus(lastAngles.get()).div(Constants.loopPeriodSecs));

        var voltages = dynamics.feedforward(angles.get(), velocities);
        shoulderVoltageFeedforward = voltages.get(0, 0);
        elbowVoltageFeedforward = voltages.get(1, 0);
        shoulderVoltageFeedback = shoulderFeedback.calculate(shoulderAngle, angles.get().get(0, 0));
        elbowVoltageFeedback = elbowFeedback.calculate(elbowAngle, angles.get().get(1, 0));
        io.setShoulderVoltage(
            MathUtil.clamp(
                shoulderVoltageFeedforward
                    + applyKs(shoulderVoltageFeedback, shoulderKs.get(), shoulderKsDeadband.get()),
                -maxVoltageNoTrajectory,
                maxVoltageNoTrajectory));
        io.setElbowVoltage(
            MathUtil.clamp(
                elbowVoltageFeedforward
                    + applyKs(elbowVoltageFeedback, elbowKs.get(), elbowKsDeadband.get()),
                -maxVoltageNoTrajectory,
                maxVoltageNoTrajectory));
      } else {
        io.setShoulderVoltage(0.0);
        io.setElbowVoltage(0.0);
      }
      wristEffectiveArmPose = setpointPose;
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
        wristAngleSetpoint = wristFeedback.getSetpoint().position;
        wristVoltageFeedback = wristFeedback.calculate(wristAngle, wristSetpoint);
        io.setWristVoltage(wristVoltageFeedback);
      }
    }

    // Update last setpoint pose
    lastSetpointPose = setpointPose;

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
    Logger.getInstance()
        .recordOutput("Arm/Voltages/ShoulderFeedforward", shoulderVoltageFeedforward);
    Logger.getInstance().recordOutput("Arm/Voltages/ElbowFeedforward", elbowVoltageFeedforward);
    Logger.getInstance().recordOutput("Arm/Voltages/ShoulderFeedback", shoulderVoltageFeedback);
    Logger.getInstance().recordOutput("Arm/Voltages/ElbowFeedback", elbowVoltageFeedback);
    Logger.getInstance().recordOutput("Arm/Voltages/WristFeedback", wristVoltageFeedback);

    // Calculate extension percent (for acceleration limits)
    extensionPercent =
        ArmPose.Preset.HOMED
                .getPose()
                .endEffectorPosition()
                .getDistance(kinematics.forward(VecBuilder.fill(shoulderAngle, elbowAngle)))
            / (config.shoulder().length() + config.elbow().length());
    Logger.getInstance().recordOutput("Arm/ExtensionPercent", extensionPercent);

    // Trigger emergency stop if necessary
    if (Constants.getMode() != Mode.SIM && isZeroed) {
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
    Leds.getInstance().armEstopped = emergencyDisable;
  }

  /** Applies a static friction and deadband to an applied voltage. */
  private static double applyKs(double volts, double kS, double kSDeadband) {
    if (Math.abs(volts) < kSDeadband) {
      return volts;
    }
    return volts + Math.copySign(kS, volts);
  }

  /**
   * Returns the predicted time until the arm will pass through the provided region. Zero if already
   * intersecting the region and infinity if no planned motion will intersect the region.
   */
  private double getTimeUntilCollision(double[] region) {
    // Check setpoint
    if (setpointPose.endEffectorPosition().getX() >= region[0]
        && setpointPose.endEffectorPosition().getX() <= region[2]
        && setpointPose.endEffectorPosition().getY() >= region[1]
        && setpointPose.endEffectorPosition().getY() <= region[3]) {
      return 0.0;
    }
    var setpointWristPosition = setpointPose.wristPosition();
    if (setpointWristPosition.getX() >= region[0]
        && setpointWristPosition.getX() <= region[2]
        && setpointWristPosition.getY() >= region[1]
        && setpointWristPosition.getY() <= region[3]) {
      return 0.0;
    }

    // Check trajectory
    if (currentTrajectory != null && currentTrajectory.isGenerated()) {
      var points = currentTrajectory.getPoints();
      var dt = currentTrajectory.getTotalTime() / (points.size() - 1);
      var trajectoryTime =
          trajectoryTimer.get() == 0.0 || trajectoryTimer.get() < trajectoryStartWait
              ? 0.0
              : trajectoryTimer.get() - trajectoryStartWait;
      for (int i = 0; i < points.size(); i++) {
        var time = dt * i;
        if (time < trajectoryTime) {
          continue;
        }
        var endEffectorPosition = kinematics.forward(points.get(i));
        var wristPosition =
            new ArmPose(endEffectorPosition, setpointPose.globalWristAngle()).wristPosition();
        if (endEffectorPosition.getX() >= region[0]
            && endEffectorPosition.getX() <= region[2]
            && endEffectorPosition.getY() >= region[1]
            && endEffectorPosition.getY() <= region[3]) {
          return time - trajectoryTime;
        }
        if (wristPosition.getX() >= region[0]
            && wristPosition.getX() <= region[2]
            && wristPosition.getY() >= region[1]
            && wristPosition.getY() <= region[3]) {
          return time - trajectoryTime;
        }
      }
    }

    return Double.POSITIVE_INFINITY;
  }

  /** Returns whether the cube intake should be extended to avoid colliding with the arm. */
  public boolean cubeIntakeShouldExtend() {
    return getTimeUntilCollision(cubeIntakeAvoidanceRect) < avoidanceLookaheadSecs;
  }

  /** Returns the current arm setpoint. */
  public ArmPose getSetpoint() {
    return setpointPose != null ? setpointPose : ArmPose.Preset.HOMED.getPose();
  }

  /** Returns whether the current trajectory is complete. */
  public boolean isTrajectoryFinished() {
    return isTrajectoryFinished(true);
  }

  /** Returns whether the current current is complete. */
  public boolean isTrajectoryFinished(boolean includeWrist) {
    return currentTrajectory == null
        && (!includeWrist || wristFeedback.getGoal().equals(wristFeedback.getSetpoint()));
  }

  /** Returns the approximate percentage for how extended the arm is (0-1). */
  public double getExtensionPercent() {
    return extensionPercent;
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
    var parameters =
        new ArmTrajectory.Parameters(
            currentAngles.get(),
            targetAngles.get(),
            getTrajectoryConstraintOverrides(kinematics, currentAngles.get(), targetAngles.get()));
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
      if (DriverStation.isAutonomous()
          || forcePregeneratedSupplier.get()
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

    // Prevent generating new trajectories
    if (DriverStation.isAutonomous() || forcePregeneratedSupplier.get()) {
      return;
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
          new ArmTrajectory(
              new ArmTrajectory.Parameters(
                  targetAngles.get(),
                  homedAngles,
                  getTrajectoryConstraintOverrides(kinematics, targetAngles.get(), homedAngles)));
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
    double outerRadius = config.shoulder().length() + config.elbow().length(); // Approxixmate
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
   * Returns the set of constraint overrides to use for a given start and end point of a trajectory.
   * Can include overrides for front nodes, back nodes, both, or neither.
   */
  static Set<String> getTrajectoryConstraintOverrides(
      ArmKinematics kinematics, Vector<N2> initialJointPositions, Vector<N2> finalJointPositions) {
    boolean constrainFront = false;
    boolean constrainBack = false;

    Translation2d initialTranslation = kinematics.forward(initialJointPositions);
    Translation2d finalTranslation = kinematics.forward(finalJointPositions);
    if (initialTranslation.getX() > nodeConstraintMinX
        && initialTranslation.getY() > nodeConstraintMinY) {
      constrainFront = true;
    }
    if (finalTranslation.getX() > nodeConstraintMinX
        && finalTranslation.getY() > nodeConstraintMinY) {
      constrainFront = true;
    }
    if (initialTranslation.getX() < -nodeConstraintMinX
        && initialTranslation.getY() > nodeConstraintMinY) {
      constrainBack = true;
    }
    if (finalTranslation.getX() < -nodeConstraintMinX
        && finalTranslation.getY() > nodeConstraintMinY) {
      constrainBack = true;
    }

    Set<String> overrides = new HashSet<>();
    if (!constrainFront) {
      overrides.addAll(frontNodeConstraints);
    }
    if (!constrainBack) {
      overrides.addAll(backNodeConstraints);
    }
    return overrides;
  }

  /**
   * Returns the maximum reach (x coordinate relative to the arm origin) that the arm can achieve at
   * the provided height.
   */
  public double calcMaxReachAtHeight(double height) {
    return kinematics.calcMaxReachAtHeight(height);
  }
}
