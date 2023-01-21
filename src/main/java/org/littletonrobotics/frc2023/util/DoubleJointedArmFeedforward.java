// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Calculates feedforward voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 */
public class DoubleJointedArmFeedforward {
  private static final double g = 9.80665;

  public static record JointConfig(
      double mass, double length, double moi, double cgRadius, DCMotor motor) {}

  private final JointConfig joint_1;
  private final JointConfig joint_2;

  public DoubleJointedArmFeedforward(JointConfig joint_1, JointConfig joint_2) {
    this.joint_1 = joint_1;
    this.joint_2 = joint_2;
  }

  public Vector<N2> calculate(Vector<N2> position) {
    return calculate(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  public Vector<N2> calculate(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var M = new Matrix<>(N2.instance, N2.instance);
    var C = new Matrix<>(N2.instance, N2.instance);
    var Tg = new Matrix<>(N2.instance, N1.instance);

    M.set(
        0,
        0,
        joint_1.mass() * Math.pow(joint_1.cgRadius(), 2.0)
            + joint_2.mass() * (Math.pow(joint_1.length(), 2.0) + Math.pow(joint_2.cgRadius(), 2.0))
            + joint_1.moi()
            + joint_2.moi()
            + 2
                * joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0)
            + joint_2.moi()
            + joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0)
            + joint_2.moi()
            + joint_2.mass()
                * joint_1.length()
                * joint_2.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(1, 1, joint_2.mass() * Math.pow(joint_2.cgRadius(), 2.0) + joint_2.moi());
    C.set(
        0,
        0,
        -joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -joint_2.mass()
            * joint_1.length()
            * joint_2.cgRadius()
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));
    Tg.set(
        0,
        0,
        (joint_1.mass() * joint_1.cgRadius() + joint_2.mass() * joint_1.length())
                * g
                * Math.cos(position.get(0, 0))
            + joint_2.mass()
                * joint_2.cgRadius()
                * g
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        joint_2.mass()
            * joint_2.cgRadius()
            * g
            * Math.cos(position.get(0, 0) + position.get(1, 0)));

    var torque = M.times(acceleration).plus(C.times(velocity)).plus(Tg);
    return VecBuilder.fill(
        joint_1.motor().getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        joint_2.motor().getVoltage(torque.get(1, 0), velocity.get(1, 0)));
  }
}
