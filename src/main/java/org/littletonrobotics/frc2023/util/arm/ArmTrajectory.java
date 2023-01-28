// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.arm;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/** Represents a trajectory of arm states that can be generated asynchronously. */
public class ArmTrajectory {
  private final Parameters parameters;
  private double totalTime = 0.0;
  private List<Vector<N2>> points = new ArrayList<>();

  /** All of the parameters required to generate a trajectory. */
  public static record Parameters(
      Vector<N2> initialJointPositions,
      Vector<N2> finalJointPositions,
      Vector<N2> initialJointVelocities,
      Vector<N2> finalJointVelocities,
      Set<String> constraintKeys) {}

  /** Creates an arm trajectory with the given parameters. */
  public ArmTrajectory(Parameters parameters) {
    this.parameters = parameters;
    points.add(parameters.initialJointPositions());
    points.add(parameters.finalJointPositions());
  }

  /** Returns the constant parameters that can be used to generate this trajectory. */
  public Parameters getParameters() {
    return parameters;
  }

  /** Returns whether the path has been generated. */
  public boolean isGenerated() {
    return totalTime > 0.0 && points.size() > 2;
  }

  /** Sets the generated interior points. */
  public void setPoints(double totalTime, List<Vector<N2>> points) {
    this.totalTime = totalTime;
    this.points = points;
  }

  /** Returns the total time for the trajectory. */
  public double getTotalTime() {
    return this.totalTime;
  }

  /**
   * Samples the trajectory at a time, returning a matrix with the position, velocities, and
   * accelerations of the joints.
   */
  public Matrix<N2, N3> sample(double time) {
    var dt = totalTime / (points.size() - 1);

    // Get surrounding points
    int prevIndex = (int) Math.floor(time / dt);
    int nextIndex = (int) Math.ceil(time / dt);
    if (nextIndex == prevIndex) nextIndex++;
    int secondPrevIndex = prevIndex - 1;
    int secondNextIndex = nextIndex + 1;

    // Clamp to allowed indices
    prevIndex = MathUtil.clamp(prevIndex, 0, points.size() - 1);
    nextIndex = MathUtil.clamp(nextIndex, 0, points.size() - 1);
    secondPrevIndex = MathUtil.clamp(secondPrevIndex, 0, points.size() - 1);
    secondNextIndex = MathUtil.clamp(secondNextIndex, 0, points.size() - 1);

    // Calculate positions
    double position_0 =
        MathUtil.interpolate(
            points.get(prevIndex).get(0, 0), points.get(nextIndex).get(0, 0), (time % dt) / dt);
    double position_1 =
        MathUtil.interpolate(
            points.get(prevIndex).get(1, 0), points.get(nextIndex).get(1, 0), (time % dt) / dt);

    // Calculate velocities
    double velocity_0 = (points.get(nextIndex).get(0, 0) - points.get(prevIndex).get(0, 0)) / dt;
    double velocity_1 = (points.get(nextIndex).get(1, 0) - points.get(prevIndex).get(1, 0)) / dt;

    // Calculate accelerations
    double acceleration_0, acceleration_1;
    if ((time % dt) / dt < 0.5) {
      double prevVelocity_0 =
          (points.get(prevIndex).get(0, 0) - points.get(secondPrevIndex).get(0, 0)) / dt;
      double prevVelocity_1 =
          (points.get(prevIndex).get(1, 0) - points.get(secondPrevIndex).get(1, 0)) / dt;
      acceleration_0 = (velocity_0 - prevVelocity_0) / dt;
      acceleration_1 = (velocity_1 - prevVelocity_1) / dt;
    } else {
      double nextVelocity_0 =
          (points.get(secondNextIndex).get(0, 0) - points.get(nextIndex).get(0, 0)) / dt;
      double nextVelocity_1 =
          (points.get(secondNextIndex).get(1, 0) - points.get(nextIndex).get(1, 0)) / dt;
      acceleration_0 = (nextVelocity_0 - velocity_0) / dt;
      acceleration_1 = (nextVelocity_1 - velocity_1) / dt;
    }

    return new MatBuilder<>(Nat.N2(), Nat.N3())
        .fill(position_0, velocity_0, acceleration_0, position_1, velocity_1, acceleration_1);
  }
}
