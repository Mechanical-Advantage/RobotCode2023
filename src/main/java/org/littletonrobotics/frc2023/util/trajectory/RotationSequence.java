// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Map.Entry;
import java.util.TreeMap;

/**
 * Represents a sequence of timed rotations. The position and velocity of the robot is calculated to
 * minimize acceleration.
 */
public class RotationSequence {
  private final TreeMap<Double, Rotation2d> sequence = new TreeMap<>();

  /** Constructs a rotation sequence from a series of timed rotation positions. */
  public RotationSequence(TreeMap<Double, Rotation2d> sequence) {
    this.sequence.putAll(sequence);
  }

  /**
   * Sample the rotation sequence at a point in time.
   *
   * @param timeSeconds The point in time since the beginning of the rotation sequence to sample.
   * @return The state at that point in time.
   */
  public State sample(double timeSeconds) {
    double positionRadians;
    double velocityRadiansPerSec;

    Entry<Double, Rotation2d> lastPoint = sequence.floorEntry(timeSeconds);
    Entry<Double, Rotation2d> nextPoint = sequence.higherEntry(timeSeconds);
    if (lastPoint == null && nextPoint == null) { // No points in sequence
      positionRadians = 0.0;
      velocityRadiansPerSec = 0.0;
    } else if (lastPoint == null) { // Before start of sequence
      positionRadians = nextPoint.getValue().getRadians();
      velocityRadiansPerSec = 0.0;
    } else if (nextPoint == null) { // Before end of sequence
      positionRadians = lastPoint.getValue().getRadians();
      velocityRadiansPerSec = 0.0;
    } else {
      double accelerationRadiansPerSec2 =
          (4 * nextPoint.getValue().minus(lastPoint.getValue()).getRadians())
              / Math.pow(nextPoint.getKey() - lastPoint.getKey(), 2);

      if (timeSeconds < (nextPoint.getKey() + lastPoint.getKey()) / 2) { // Accelerating
        positionRadians =
            lastPoint.getValue().getRadians()
                + ((accelerationRadiansPerSec2 / 2)
                    * Math.pow(timeSeconds - lastPoint.getKey(), 2));
        velocityRadiansPerSec = (timeSeconds - lastPoint.getKey()) * accelerationRadiansPerSec2;

      } else { // Decelerating
        positionRadians =
            nextPoint.getValue().getRadians()
                - ((accelerationRadiansPerSec2 / 2)
                    * Math.pow(timeSeconds - nextPoint.getKey(), 2));
        velocityRadiansPerSec = (nextPoint.getKey() - timeSeconds) * accelerationRadiansPerSec2;
      }
    }

    // Keep position within acceptable range
    while (positionRadians > Math.PI) {
      positionRadians -= Math.PI * 2;
    }
    while (positionRadians < -Math.PI) {
      positionRadians += Math.PI * 2;
    }

    return new State(new Rotation2d(positionRadians), velocityRadiansPerSec);
  }

  /** Represents a state in a rotation sequence with a position and velocity. */
  public static class State {
    public Rotation2d position;
    public double velocityRadiansPerSec;

    public State() {
      position = new Rotation2d();
      velocityRadiansPerSec = 0.0;
    }

    /**
     * Constructs a State with the specified parameters.
     *
     * @param position The position at this point in the rotation sequence.
     * @param velocityRadiansPerSec The velocity at this point in the rotation sequence.
     */
    public State(Rotation2d position, double velocityRadiansPerSec) {
      this.position = position;
      this.velocityRadiansPerSec = velocityRadiansPerSec;
    }
  }
}
