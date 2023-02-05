// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeVisualizer extends SubsystemBase {
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(20.0, 50.0);
  private TrapezoidProfile.State cubeState = new TrapezoidProfile.State(Math.PI / 2, 0.0);
  private TrapezoidProfile.State coneState = new TrapezoidProfile.State(Math.PI / 2, 0.0);
  private final Supplier<Boolean> cubeExtend;
  private final Supplier<Boolean> coneExtend;

  public IntakeVisualizer(Supplier<Boolean> cubeExtend, Supplier<Boolean> coneExtend) {
    this.cubeExtend = cubeExtend;
    this.coneExtend = coneExtend;
  }

  public void periodic() {
    cubeState =
        new TrapezoidProfile(
                constraints,
                new TrapezoidProfile.State(cubeExtend.get() ? 0.0 : Math.PI / 2, 0.0),
                cubeState)
            .calculate(Constants.loopPeriodSecs);
    coneState =
        new TrapezoidProfile(
                constraints,
                new TrapezoidProfile.State(coneExtend.get() ? Math.PI : Math.PI / 2, 0.0),
                coneState)
            .calculate(Constants.loopPeriodSecs);

    Logger.getInstance()
        .recordOutput(
            "Mechanisms3d/Intakes",
            new Pose3d(0.28, 0.0, 0.197, new Rotation3d(0.0, -cubeState.position, 0.0)),
            new Pose3d(
                -0.2,
                0.0,
                0.236,
                new Rotation3d(Math.PI, 0.0, 0.0)
                    .plus(new Rotation3d(0.0, -coneState.position, 0.0))));
  }
}
