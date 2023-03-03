// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.cubeintake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class CubeIntake extends SubsystemBase {
  private CubeIntakeIO io;
  private final CubeIntakeIOInputsAutoLogged inputs = new CubeIntakeIOInputsAutoLogged();

  private static final Translation2d rootPosition = new Translation2d(0.28, 0.197);
  private Mechanism2d mechanism;
  private MechanismRoot2d mechanismRoot;
  private MechanismLigament2d mechanismLigament;

  private boolean isZeroed = false;
  private double absoluteAngleOffset = 0.0;
  private boolean isRunning = false;
  private Supplier<Boolean> enableRollerSupplier = () -> true;
  private boolean lastCoast = false;

  private Supplier<Boolean> forceExtendSupplier = () -> false;
  private Supplier<Boolean> coastSupplier = () -> false;

  private static final LoggedTunableNumber neutralPositionDegrees =
      new LoggedTunableNumber("CubeIntake/NeutralPositionDegrees");
  private static final LoggedTunableNumber deployPositionDegrees =
      new LoggedTunableNumber("CubeIntake/DeployPositionDegrees");
  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("CubeIntake/RollerVolts");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("CubeIntake/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("CubeIntake/kD");
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("CubeIntake/MaxVelocity");
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("CubeIntake/MaxAcceleration");
  private ProfiledPIDController controller =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        neutralPositionDegrees.initDefault(98.0);
        deployPositionDegrees.initDefault(5.0);
        rollerVolts.initDefault(12.0);
        kP.initDefault(6.0);
        kD.initDefault(0.0);
        maxVelocity.initDefault(50.0);
        maxAcceleration.initDefault(100.0);
        break;
      case ROBOT_SIMBOT:
        neutralPositionDegrees.initDefault(90.0);
        deployPositionDegrees.initDefault(0.0);
        rollerVolts.initDefault(8.0);
        kP.initDefault(30.0);
        kD.initDefault(2.0);
        maxVelocity.initDefault(10.0);
        maxAcceleration.initDefault(15.0);
        break;
      default:
        break;
    }
  }

  /** Creates a new CubeIntake. */
  public CubeIntake(CubeIntakeIO io) {
    this.io = io;
    io.setBrakeMode(true, false);

    // Create mechanism
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("CubeIntake", 2.0 + rootPosition.getX(), rootPosition.getY());
    mechanismLigament =
        mechanismRoot.append(
            new MechanismLigament2d("IntakeArm", 0.35, 90, 4, new Color8Bit(Color.kLightGreen)));
  }

  public void setSuppliers(Supplier<Boolean> forceExtendSupplier, Supplier<Boolean> coastSupplier) {
    this.forceExtendSupplier = forceExtendSupplier;
    this.coastSupplier = coastSupplier;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("CubeIntake", inputs);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get());
      controller.setD(kD.get());
    }
    if (maxVelocity.hasChanged(hashCode()) && maxAcceleration.hasChanged(hashCode())) {
      controller.setConstraints(
          new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    // Update coast mode
    boolean coast = coastSupplier.get() && DriverStation.isDisabled();
    if (coast != lastCoast) {
      lastCoast = coast;
      io.setBrakeMode(!coast, false);
    }

    // Zero with absolute encoder
    if (!isZeroed) {
      absoluteAngleOffset = inputs.armAbsolutePositionRad - inputs.armRelativePositionRad;
      isZeroed = true;
    }

    // Get measured positions
    double angle = inputs.armRelativePositionRad + absoluteAngleOffset;
    mechanismLigament.setAngle(new Rotation2d(angle));
    Logger.getInstance().recordOutput("Mechanism2d/CubeIntake", mechanism);
    Logger.getInstance().recordOutput("Mechanism3d/CubeIntake", getPose3d(angle));
    Logger.getInstance().recordOutput("CubeIntake/AngleRadians", angle);
    Logger.getInstance()
        .recordOutput("CubeIntake/AngleSetpointRadians", controller.getSetpoint().position);
    Logger.getInstance().recordOutput("CubeIntake/AngleGoalRadians", controller.getGoal().position);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setArmVoltage(0.0);
      io.setRollerVoltage(0.0);
      controller.reset(angle);
      isRunning = false;

    } else {
      // Run controller when enabled
      if (isRunning || forceExtendSupplier.get()) {
        controller.setGoal(Units.degreesToRadians(deployPositionDegrees.get()));
      } else {
        controller.setGoal(Units.degreesToRadians(neutralPositionDegrees.get()));
      }
      io.setArmVoltage(controller.calculate(angle));

      // Run roller
      io.setRollerVoltage(isRollerRunning() ? rollerVolts.get() : 0.0);
    }
  }

  /** Returns whether the roller is running. */
  public boolean isRollerRunning() {
    return isRunning
        && controller.getGoal().equals(controller.getSetpoint())
        && enableRollerSupplier.get();
  }

  /** Returns the 3D pose of the intake for visualization. */
  private Pose3d getPose3d(double angle) {
    return new Pose3d(
        rootPosition.getX(), 0.0, rootPosition.getY(), new Rotation3d(0.0, -angle, 0.0));
  }

  /** Command factory to extend and run the roller. */
  public Command runCommand() {
    return runCommand(() -> true);
  }

  /** Command factory to extend and run the roller. */
  public Command runCommand(Supplier<Boolean> enableRollerSupplier) {
    return startEnd(
        () -> {
          isRunning = true;
          this.enableRollerSupplier = enableRollerSupplier;
        },
        () -> isRunning = false);
  }
}
