// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/**
 * Calculates feedforward voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 */
public class ArmFeedforward {
  private static final double g = 9.80665;
  private final ArmConfig config;

  public ArmFeedforward(ArmConfig config) {
    this.config = config;
  }

  /** Calculates the joint voltages based on the joint positions. */
  public Vector<N2> calculate(Vector<N2> position) {
    return calculate(position, VecBuilder.fill(0.0, 0.0));
  }

  /** Calculates the joint voltages based on the joint positions and velocities. */
  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity) {
    return calculate(position, velocity, VecBuilder.fill(0.0, 0.0));
  }

  /** Calculates the joint voltages based on the full joint states as a matrix. */
  public Vector<N2> calculate(Matrix<N2, N3> state) {
    return calculate(
        new Vector<>(state.extractColumnVector(0)),
        new Vector<>(state.extractColumnVector(1)),
        new Vector<>(state.extractColumnVector(2)));
  }

  /** Calculates the joint voltages based on the full joint states as vectors. */
  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var M = new Matrix<>(N2.instance, N2.instance);
    var C = new Matrix<>(N2.instance, N2.instance);
    var Tg = new Matrix<>(N2.instance, N1.instance);

    M.set(
        0,
        0,
        config.shoulder().mass() * Math.pow(config.shoulder().cgRadius(), 2.0)
            + config.elbow().mass()
                * (Math.pow(config.shoulder().length(), 2.0)
                    + Math.pow(config.elbow().cgRadius(), 2.0))
            + config.shoulder().moi()
            + config.elbow().moi()
            + 2
                * config.elbow().mass()
                * config.shoulder().length()
                * config.elbow().cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        config.elbow().mass() * Math.pow(config.elbow().cgRadius(), 2.0)
            + config.elbow().moi()
            + config.elbow().mass()
                * config.shoulder().length()
                * config.elbow().cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        config.elbow().mass() * Math.pow(config.elbow().cgRadius(), 2.0)
            + config.elbow().moi()
            + config.elbow().mass()
                * config.shoulder().length()
                * config.elbow().cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        1,
        config.elbow().mass() * Math.pow(config.elbow().cgRadius(), 2.0) + config.elbow().moi());
    C.set(
        0,
        0,
        -config.elbow().mass()
            * config.shoulder().length()
            * config.elbow().cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        config.elbow().mass()
            * config.shoulder().length()
            * config.elbow().cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -config.elbow().mass()
            * config.shoulder().length()
            * config.elbow().cgRadius()
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));
    Tg.set(
        0,
        0,
        (config.shoulder().mass() * config.shoulder().cgRadius()
                    + config.elbow().mass() * config.shoulder().length())
                * g
                * Math.cos(position.get(0, 0))
            + config.elbow().mass()
                * config.elbow().cgRadius()
                * g
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        config.elbow().mass()
            * config.elbow().cgRadius()
            * g
            * Math.cos(position.get(0, 0) + position.get(1, 0)));

    var torque = M.times(acceleration).plus(C.times(velocity)).plus(Tg);
    return VecBuilder.fill(
        config.shoulder().motor().getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        config.elbow().motor().getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }
}
