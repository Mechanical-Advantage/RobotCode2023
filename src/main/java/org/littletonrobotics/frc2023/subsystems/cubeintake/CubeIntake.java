// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.cubeintake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private Mechanism2d mechanism;
  private MechanismRoot2d mechanismRoot;
  private MechanismLigament2d mechanismLigament;

  private boolean isZeroed = false;
  private double absoluteAngleOffset = 0.0;
  private boolean isRunning = false;
  private Supplier<Boolean> forceExtendSupplier = () -> false;

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
      case ROBOT_SIMBOT:
        kP.initDefault(30.0);
        kD.initDefault(2.0);
        maxVelocity.initDefault(10.0);
        maxAcceleration.initDefault(50.0);
        break;
      default:
        break;
    }
  }

  /** Creates a new CubeIntake. */
  public CubeIntake(CubeIntakeIO io) {
    this.io = io;

    // Create mechanism
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("CubeIntake", 2.28, 0.197);
    mechanismLigament =
        mechanismRoot.append(
            new MechanismLigament2d("IntakeArm", 0.5, 90, 4, new Color8Bit(Color.kLightGreen)));
  }

  public void setForceExtendSupplier(Supplier<Boolean> supplier) {
    forceExtendSupplier = supplier;
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

    // Zero with absolute encoder
    if (!isZeroed) {
      absoluteAngleOffset = inputs.armAbsolutePositionRad - inputs.armRelativePositionRad;
      isZeroed = true;
    }

    // Get measured positions
    double angle = inputs.armRelativePositionRad + absoluteAngleOffset;
    mechanismLigament.setAngle(new Rotation2d(angle));
    Logger.getInstance().recordOutput("Mechanism2d/CubeIntake", mechanism);
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
        controller.setGoal(Math.PI / 2);
      }
      io.setArmVoltage(controller.calculate(angle));

      // Run roller
      io.setRollerVoltage(isRunning ? rollerVolts.get() : 0.0);
    }
  }

  /** Command factory to extend and run the roller. */
  public Command run() {
    return startEnd(() -> isRunning = true, () -> isRunning = false);
  }
}
