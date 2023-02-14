// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.coneintake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ConeIntake extends SubsystemBase {
  private ConeIntakeIO io;
  private final ConeIntakeIOInputsAutoLogged inputs = new ConeIntakeIOInputsAutoLogged();

  private static final Translation2d rootPosition = new Translation2d(-0.282, 0.195);
  private Mechanism2d mechanism;
  private MechanismRoot2d mechanismRoot;
  private MechanismLigament2d mechanismLigament;

  private boolean isZeroed = false;
  private double absoluteAngleOffset = 0.0;
  private Mode mode = Mode.NEUTRAL;
  private Supplier<Boolean> forceExtendSupplier = () -> false;

  private static final LoggedTunableNumber neutralPositionDegrees =
      new LoggedTunableNumber("ConeIntake/NeutralPositionDegrees");
  private static final LoggedTunableNumber deployPositionDegrees =
      new LoggedTunableNumber("ConeIntake/DeployPositionDegrees");
  private static final LoggedTunableNumber handoffPositionDegrees =
      new LoggedTunableNumber("ConeIntake/HandoffPositionDegrees");
  private static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("ConeIntake/IntakeVolts");
  private static final LoggedTunableNumber ejectVolts =
      new LoggedTunableNumber("ConeIntake/EjectVolts");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("ConeIntake/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("ConeIntake/kD");
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("ConeIntake/MaxVelocity");
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("ConeIntake/MaxAcceleration");
  private ProfiledPIDController controller =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  static {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        neutralPositionDegrees.initDefault(90.0);
        deployPositionDegrees.initDefault(0.0);
        handoffPositionDegrees.initDefault(55.0);
        intakeVolts.initDefault(8.0);
        ejectVolts.initDefault(-8.0);
        kP.initDefault(30.0);
        kD.initDefault(2.0);
        maxVelocity.initDefault(10.0);
        maxAcceleration.initDefault(50.0);
        break;
      default:
        break;
    }
  }

  /** Creates a new ConeIntake. */
  public ConeIntake(ConeIntakeIO io) {
    this.io = io;

    // Create mechanism
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("ConeIntake", 2.0 + rootPosition.getX(), rootPosition.getY());
    mechanismLigament =
        mechanismRoot.append(
            new MechanismLigament2d("IntakeArm", 0.35, 90, 4, new Color8Bit(Color.kLightGreen)));

    // Default to neutral
    setDefaultCommand(run(() -> setMode(Mode.NEUTRAL)));
  }

  public void setForceExtendSupplier(Supplier<Boolean> supplier) {
    forceExtendSupplier = supplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("ConeIntake", inputs);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get());
      controller.setD(kD.get());
    }
    if (maxVelocity.hasChanged(hashCode()) && maxAcceleration.hasChanged(hashCode())) {
      controller.setConstraints(
          new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    // Zero with absolute encoder
    if (!isZeroed) {
      absoluteAngleOffset = inputs.armAbsolutePositionRad - inputs.armRelativePositionRad;
      isZeroed = true;
    }

    // Get measured positions
    double angle = inputs.armRelativePositionRad + absoluteAngleOffset;
    mechanismLigament.setAngle(new Rotation2d(angle));
    Logger.getInstance().recordOutput("Mechanism2d/ConeIntake", mechanism);
    Logger.getInstance().recordOutput("Mechanism3d/ConeIntake", getPose3d(angle));
    Logger.getInstance().recordOutput("ConeIntake/AngleRadians", angle);
    Logger.getInstance()
        .recordOutput("ConeIntake/AngleSetpointRadians", controller.getSetpoint().position);
    Logger.getInstance().recordOutput("ConeIntake/AngleGoalRadians", controller.getGoal().position);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setArmVoltage(0.0);
      io.setRollerVoltage(0.0);
      controller.reset(angle);
      mode = Mode.NEUTRAL;

    } else {
      // Run controller when enabled
      switch (mode) {
        case NEUTRAL:
          controller.setGoal(
              Units.degreesToRadians(
                  forceExtendSupplier.get()
                      ? deployPositionDegrees.get()
                      : neutralPositionDegrees.get()));
          io.setRollerVoltage(0.0);
          break;
        case INTAKING:
          controller.setGoal(Units.degreesToRadians(deployPositionDegrees.get()));
          io.setRollerVoltage(intakeVolts.get());
          break;
        case HANDOFF_START:
          controller.setGoal(Units.degreesToRadians(handoffPositionDegrees.get()));
          io.setRollerVoltage(0.0);
          break;
        case HANDOFF_EJECT:
          controller.setGoal(Units.degreesToRadians(neutralPositionDegrees.get()));
          io.setRollerVoltage(ejectVolts.get());
          break;
      }
      io.setArmVoltage(controller.calculate(angle));
    }
  }

  /** Returns the 3D pose of the intake for visualization. */
  private Pose3d getPose3d(double angle) {
    return new Pose3d(
            rootPosition.getX(), 0.0, rootPosition.getY(), new Rotation3d(0.0, 0.0, Math.PI))
        .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0.0, -angle, 0.0)));
  }

  /** Sets the current mode of operation. */
  public void setMode(Mode mode) {
    this.mode = mode;
  }

  /** Command factory to set the current mode of operation. */
  public Command setModeCommand(Mode mode) {
    return runOnce(() -> this.mode = mode);
  }

  public static enum Mode {
    NEUTRAL,
    INTAKING,
    HANDOFF_START,
    HANDOFF_EJECT
  }
}
