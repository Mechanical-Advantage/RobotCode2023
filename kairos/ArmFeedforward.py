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
    _shoulder: JointConfig
    _elbow: JointConfig

    def __init__(self, shoulder: JointConfig, elbow: JointConfig):
        self._shoulder = shoulder
        self._elbow = elbow

    def calculate(self, position, velocity, acceleration):
        M = [[0, 0], [0, 0]]
        C = [[0, 0], [0, 0]]
        Tg = [0, 0]

        M[0][0] = (
            self._shoulder.mass * (self._shoulder.cgRadius**2.0)
            + self._elbow.mass
            * ((self._shoulder.length**2.0) + (self._elbow.cgRadius**2.0))
            + self._shoulder.moi
            + self._elbow.moi
            + 2
            * self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * cos(position[1])
        )
        M[1][0] = (
            self._elbow.mass * (self._elbow.cgRadius**2)
            + self._elbow.moi
            + self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * cos(position[1])
        )
        M[0][1] = (
            self._elbow.mass * (self._elbow.cgRadius**2)
            + self._elbow.moi
            + self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * cos(position[1])
        )
        M[1][1] = self._elbow.mass * (self._elbow.cgRadius**2) + self._elbow.moi

        C[0][0] = (
            -self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * sin(position[1])
            * velocity[1]
        )
        C[1][0] = (
            self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * sin(position[1])
            * velocity[0]
        )
        C[0][1] = (
            -self._elbow.mass
            * self._shoulder.length
            * self._elbow.cgRadius
            * sin(position[1])
            * (velocity[0] + velocity[1])
        )

        Tg[0] = (
            self._shoulder.mass * self._shoulder.cgRadius
            + self._elbow.mass * self._shoulder.length
        ) * self._g * cos(
            position[0]
        ) + self._elbow.mass * self._elbow.cgRadius * self._g * cos(
            position[0] + position[1]
        )
        Tg[1] = (
            self._elbow.mass
            * self._elbow.cgRadius
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
            self._shoulder.motor.getVoltage(torque[0], velocity[0]),
            self._elbow.motor.getVoltage(torque[1], velocity[1]),
        )
