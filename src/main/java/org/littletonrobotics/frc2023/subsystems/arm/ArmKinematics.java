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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import java.util.Optional;

/**
 * Converts between joint angles and the end effector position.
 *
 * <p>https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
 */
public class ArmKinematics {
  private final ArmConfig config;

  public ArmKinematics(ArmConfig config) {
    this.config = config;
  }

  /** Converts joint angles to the end effector position. */
  public Translation2d forward(Vector<N2> angles) {
    return new Translation2d(
        config.origin().getX()
            + config.shoulder().length() * Math.cos(angles.get(0, 0))
            + config.elbow().length() * Math.cos(angles.get(0, 0) + angles.get(1, 0)),
        config.origin().getY()
            + config.shoulder().length() * Math.sin(angles.get(0, 0))
            + config.elbow().length() * Math.sin(angles.get(0, 0) + angles.get(1, 0)));
  }

  /** Converts the end effector position to joint angles. */
  public Optional<Vector<N2>> inverse(Translation2d position) {
    Translation2d relativePosition = position.minus(config.origin());

    // Flip when X is negative
    boolean isFlipped = relativePosition.getX() < 0.0;
    if (isFlipped) {
      relativePosition = new Translation2d(-relativePosition.getX(), relativePosition.getY());
    }

    // Calculate angles
    double elbowAngle =
        -Math.acos(
            (Math.pow(relativePosition.getX(), 2)
                    + Math.pow(relativePosition.getY(), 2)
                    - Math.pow(config.shoulder().length(), 2)
                    - Math.pow(config.elbow().length(), 2))
                / (2 * config.shoulder().length() * config.elbow().length()));
    if (Double.isNaN(elbowAngle)) {
      return Optional.empty();
    }
    double shoulderAngle =
        Math.atan(relativePosition.getY() / relativePosition.getX())
            - Math.atan(
                (config.elbow().length() * Math.sin(elbowAngle))
                    / (config.shoulder().length()
                        + config.elbow().length() * Math.cos(elbowAngle)));

    // Invert shoulder angle if invalid
    Translation2d testPosition =
        forward(VecBuilder.fill(shoulderAngle, elbowAngle)).minus(config.origin());
    if (testPosition.getDistance(relativePosition) > 1e-3) {
      shoulderAngle += Math.PI;
    }

    // Flip angles
    if (isFlipped) {
      shoulderAngle = Math.PI - shoulderAngle;
      elbowAngle = -elbowAngle;
    }

    // Wrap angles to correct ranges
    shoulderAngle = MathUtil.inputModulus(shoulderAngle, -Math.PI, Math.PI);
    elbowAngle = MathUtil.inputModulus(elbowAngle, 0.0, Math.PI * 2.0);

    // Exit if outside valid ranges for the joints
    if (shoulderAngle < config.shoulder().minAngle()
        || shoulderAngle > config.shoulder().maxAngle()
        || elbowAngle < config.elbow().minAngle()
        || elbowAngle > config.elbow().maxAngle()) {
      return Optional.empty();
    }

    // Return result
    return Optional.of(VecBuilder.fill(shoulderAngle, elbowAngle));
  }

  /**
   * Returns the maximum reach (x coordinate relative to the arm origin) that the arm can achieve at
   * the provided height.
   */
  public double calcMaxReachAtHeight(double height) {
    // Set the elbow angle equation to the max angle and solve for x
    return Math.sqrt(
            Math.cos(-config.elbow().maxAngle())
                    * 2
                    * config.shoulder().length()
                    * config.elbow().length()
                - Math.pow(height - config.origin().getY(), 2)
                + Math.pow(config.shoulder().length(), 2)
                + Math.pow(config.elbow().length(), 2))
        - 1e-6; // Shift back to ensure this is still valid after rounding errors
  }
}
