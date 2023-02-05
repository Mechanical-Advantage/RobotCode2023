# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math

from casadi import *


class DCMotor:
    _nominalVoltageVolts: float = 0
    _stallTorqueNewtonMeters: float = 0
    _stallCurrentAmps: float = 0
    _freeCurrentAmps: float = 0
    _freeSpeedRadPerSec: float = 0
    _rOhms: float = 0
    _KvRadPerSecPerVolt: float = 0
    _KtNMPerAmp: float = 0

    def __init__(
        self,
        nominalVoltageVolts: float,
        stallTorqueNewtonMeters: float,
        stallCurrentAmps: float,
        freeCurrentAmps: float,
        freeSpeedRadPerSec: float,
        numMotors: int,
    ):
        self._nominalVoltageVolts = nominalVoltageVolts
        self._stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors
        self._stallCurrentAmps = stallCurrentAmps * numMotors
        self._freeCurrentAmps = freeCurrentAmps * numMotors
        self._freeSpeedRadPerSec = freeSpeedRadPerSec

        self._rOhms = nominalVoltageVolts / self._stallCurrentAmps
        self._KvRadPerSecPerVolt = freeSpeedRadPerSec / (
            nominalVoltageVolts - self._rOhms * self._freeCurrentAmps
        )
        self._KtNMPerAmp = self._stallTorqueNewtonMeters / self._stallCurrentAmps

    def getVoltage(self, torqueNm, speedRadiansPerSec):
        return (
            1.0 / self._KvRadPerSecPerVolt * speedRadiansPerSec
            + 1.0 / self._KtNMPerAmp * self._rOhms * torqueNm
        )

    def withReduction(self, gearboxReduction: float):
        return DCMotor(
            self._nominalVoltageVolts,
            self._stallTorqueNewtonMeters * gearboxReduction,
            self._stallCurrentAmps,
            self._freeCurrentAmps,
            self._freeSpeedRadPerSec / gearboxReduction,
            1,
        )

    @classmethod
    def getNEO(self, numMotors):
        rpm = 5676
        return DCMotor(12, 2.6, 105, 1.8, rpm * math.pi / (60.0 / 2.0), numMotors)

    @classmethod
    def getNEO550(self, numMotors):
        rpm = 11000
        return DCMotor(12, 0.97, 100, 1.4, rpm * math.pi / (60.0 / 2.0), numMotors)
