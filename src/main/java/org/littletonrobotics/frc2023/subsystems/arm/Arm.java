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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final double trajectoryCacheMarginRadians = 0.02;

  private final ArmIO io;
  private final ArmSolverIO solverIo;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmSolverIOInputsAutoLogged solverInputs = new ArmSolverIOInputsAutoLogged();

  private static final String configFilename = "arm_config.json";
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
        shoulderKp.initDefault(10.0);
        shoulderKd.initDefault(0.0);
        elbowKp.initDefault(10.0);
        elbowKd.initDefault(0.0);
        wristKp.initDefault(30.0);
        wristKd.initDefault(0.0);
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
      FORWARD(new ArmPose(new Translation2d(1.0, 1.0), new Rotation2d(-Math.PI / 3))),
      BACKWARD(new ArmPose(new Translation2d(-1.0, 1.0), new Rotation2d(-2 * Math.PI / 3))),
      FORWARD_UP(new ArmPose(new Translation2d(0.5, 1.5), new Rotation2d())),
      BACKWARD_UP(new ArmPose(new Translation2d(-0.5, 1.5), new Rotation2d(Math.PI)));

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

    // Calculate homed pose
    ArmPose.Preset.HOMED.setPose(
        new ArmPose(
            new Translation2d(
                config.origin().getX(),
                config.origin().getY() + config.shoulder().length() - config.elbow().length()),
            new Rotation2d(-Math.PI / 2)));

    // Create visualizers
    visualizerMeasured = new ArmVisualizer(config, "Measured", null);
    visualizerSetpoint = new ArmVisualizer(config, "Setpoint", new Color8Bit(Color.kOrange));

    // Add preset trajectories to queue
    for (var preset0 : ArmPose.Preset.values()) {
      for (var preset1 : ArmPose.Preset.values()) {
        if (!preset0.equals(preset1)) {
          Optional<Vector<N2>> preset0Angles =
              kinematics.inverse(preset0.getPose().endEffectorPosition());
          Optional<Vector<N2>> preset1Angles =
              kinematics.inverse(preset1.getPose().endEffectorPosition());
          if (preset0Angles.isPresent() && preset1Angles.isPresent()) {
            var parameters = new ArmTrajectory.Parameters(preset0Angles.get(), preset1Angles.get());
            allTrajectories.put(parameters.hashCode(), new ArmTrajectory(parameters));
          }
        }
      }
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
    if (trajectoryCountGenerated
        >= ArmPose.Preset.values().length * (ArmPose.Preset.values().length - 1)) {
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
        double wristSetpoint =
            wristEffectiveArmPose
                .globalWristAngle()
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
  }

  public void setPose(ArmPose.Preset preset) {
    setPose(preset.getPose());
  }

  public void setPose(ArmPose pose) {
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

  public void shiftPose(Translation2d shift) {
    if (shift.getNorm() > 0.0) {
      var newTranslation = setpointPose.endEffectorPosition().plus(shift);
      var angles = kinematics.inverse(newTranslation);
      if (angles.isPresent()) {
        currentTrajectory = null;
        setpointPose = new ArmPose(newTranslation, setpointPose.globalWristAngle());
      }
    }
  }
}
