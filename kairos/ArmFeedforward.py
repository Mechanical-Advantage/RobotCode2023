# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from dataclasses import dataclass

from casadi import *

from DCMotor import DCMotor


@dataclass
class JointConfig:
    mass: float
    length: float
    moi: float
    cgRadius: float
    motor: DCMotor


class ArmFeedforward:
    _g = 9.80665
    _joint_1: JointConfig
    _joint_2: JointConfig

    def __init__(self, joint_1: JointConfig, joint_2: JointConfig):
        self._joint_1 = joint_1
        self._joint_2 = joint_2

    def calculate(self, position, velocity, acceleration):
        M = [[0, 0], [0, 0]]
        C = [[0, 0], [0, 0]]
        Tg = [0, 0]

        M[0][0] = (
            self._joint_1.mass * (self._joint_1.cgRadius**2.0)
            + self._joint_2.mass
            * ((self._joint_1.length**2.0) + (self._joint_2.cgRadius**2.0))
            + self._joint_1.moi
            + self._joint_2.moi
            + 2
            * self._joint_2.mass
            * self._joint_1.length
            * self._joint_2.cgRadius
            * cos(position[1])
        )
        M[1][0] = (
            self._joint_2.mass * (self._joint_2.cgRadius**2)
            + self._joint_2.moi
            + self._joint_2.mass
            * self._joint_1.length
            * self._joint_2.cgRadius
            * cos(position[1])
        )
        M[0][1] = (
            self._joint_2.mass * (self._joint_2.cgRadius**2)
            + self._joint_2.moi
            + self._joint_2.mass
            * self._joint_1.length
            * self._joint_2.cgRadius
            * cos(position[1])
        )
        M[1][1] = self._joint_2.mass * (self._joint_2.cgRadius**2) + self._joint_2.moi

        C[0][0] = (
            -self._joint_2.mass
            * self._joint_1.length
            * self._joint_2.cgRadius
            * sin(position[1])
            * velocity[1]
        )
        C[1][0] = (
            self._joint_2.mass
            * self._joint_1.length
            * self._joint_2.cgRadius
            * sin(position[1])
            * velocity[0]
        )
        C[0][1] = (
            -self._joint_2.mass
            * self._joint_1.length
            * self._joint_2.cgRadius
            * sin(position[1])
            * (velocity[0] + velocity[1])
        )

        Tg[0] = (
            self._joint_1.mass * self._joint_1.cgRadius
            + self._joint_2.mass * self._joint_1.length
        ) * self._g * cos(
            position[0]
        ) + self._joint_2.mass * self._joint_2.cgRadius * self._g * cos(
            position[0] + position[1]
        )
        Tg[1] = (
            self._joint_2.mass
            * self._joint_2.cgRadius
            * self._g
            * cos(position[0] + position[1])
        )

        M_times_acceleration = (
            M[0][0] * acceleration[0] + M[0][1] * acceleration[1],
            M[1][0] * acceleration[0] + M[1][1] * acceleration[1],
        )
        C_times_velocity = (
            C[0][0] * velocity[0] + C[0][1] * velocity[1],
            C[1][0] * velocity[0] + C[1][1] * velocity[1],
        )
        torque = (
            M_times_acceleration[0] + C_times_velocity[0] + Tg[0],
            M_times_acceleration[1] + C_times_velocity[1] + Tg[1],
        )
        return (
            self._joint_1.motor.getVoltage(torque[0], velocity[0]),
            self._joint_2.motor.getVoltage(torque[1], velocity[1]),
        )
