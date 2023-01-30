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
import java.util.Set;
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
  private final ArmFeedforward ffModel;
  private final ArmKinematics kinematics;

  private final ArmMechanism2d mechanismMeasured;
  private final ArmMechanism2d mechanismSetpoint;

  private boolean isZeroed = false;
  private double shoulderAngleOffset = 0.0;
  private double elbowAngleOffset = 0.0;
  private double wristAngleOffset = 0.0;
  private double shoulderAngle = 0.0;
  private double elbowAngle = 0.0;
  private double wristAngle = 0.0;

  private final Map<Integer, ArmTrajectory> allTrajectories = new HashMap<>();
  private ArmTrajectory currentTrajectory = null;
  private ArmPose setpointPose = null;
  private ArmPose queuedPose = null;
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
        shoulderKp.initDefault(0.0);
        shoulderKd.initDefault(0.0);
        elbowKp.initDefault(0.0);
        elbowKd.initDefault(0.0);
        wristKp.initDefault(0.0);
        wristKd.initDefault(0.0);
        wristMaxVelocity.initDefault(0.0);
        wristMaxAcceleration.initDefault(0.0);
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
    config = ArmConfig.fromJson(configFile);
    io.setConfig(config);
    solverIo.setConfig(configJson);
    ffModel = new ArmFeedforward(config);
    kinematics = new ArmKinematics(config);

    // Create mechanisms
    mechanismMeasured = new ArmMechanism2d(config, "Measured", null);
    mechanismSetpoint = new ArmMechanism2d(config, "Setpoint", new Color8Bit(Color.kOrange));
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
    mechanismMeasured.update(shoulderAngle, elbowAngle, wristAngle);

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

    // Check if trajectory is finished
    if (trajectoryTimer.hasElapsed(currentTrajectory.getTotalTime())) {
      trajectoryTimer.stop();
      trajectoryTimer.reset();
      currentTrajectory = null;
      setpointPose = queuedPose;
    }

    // Run shoulder and elbow
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
      var voltages = ffModel.calculate(state);
      io.setShoulderVoltage(
          voltages.get(0, 0) + shoulderFeedback.calculate(shoulderAngle, state.get(0, 0)));
      io.setElbowVoltage(voltages.get(1, 0) + elbowFeedback.calculate(elbowAngle, state.get(1, 0)));
      setpointPose =
          new ArmPose(
              kinematics.forward(new Vector<>(state.extractColumnVector(0))),
              queuedPose.globalWristAngle());

    } else {
      // Go to setpoint
      Optional<Vector<N2>> angles = kinematics.inverse(setpointPose.endEffectorPosition());
      if (angles.isPresent()) {
        io.setShoulderVoltage(shoulderFeedback.calculate(shoulderAngle, angles.get().get(0, 0)));
        io.setElbowVoltage(elbowFeedback.calculate(elbowAngle, angles.get().get(1, 0)));
      }
    }
  }

  public void setTarget(ArmPose pose) {
    // Get parameters
    Optional<Vector<N2>> angles = kinematics.inverse(pose.endEffectorPosition());
    if (angles.isEmpty()) {
      return;
    }
    var parameters =
        new ArmTrajectory.Parameters(
            VecBuilder.fill(shoulderAngle, elbowAngle),
            angles.get(),
            VecBuilder.fill(inputs.shoulderVelocityRadPerSec, inputs.elbowVelocityRadPerSec),
            VecBuilder.fill(0.0, 0.0),
            Set.of());

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
          && Math.abs(finalDiff.get(1, 0)) < trajectoryCacheMarginRadians) {
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
    solverIo.request(parameters);
  }

  /** Represents a target position for the arm. */
  public static record ArmPose(Translation2d endEffectorPosition, Rotation2d globalWristAngle) {}
}
