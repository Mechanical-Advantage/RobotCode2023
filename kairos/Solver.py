# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from casadi import *

from ArmFeedforward import ArmFeedforward, JointConfig
from DCMotor import DCMotor


class Solver:
    def __init__(self, config: str):
        self._config = config

        # Create solver
        opti = Opti()
        self._opti = opti
        opti.solver("ipopt")

        # Get constants from config
        n = config["solver"]["interiorPoints"]
        max_voltage = config["solver"]["maxVoltage"]
        ff_model = ArmFeedforward(
            JointConfig(
                config["shoulder"]["mass"],
                config["shoulder"]["length"],
                config["shoulder"]["moi"],
                config["shoulder"]["cgRadius"],
                self._get_motor(config["shoulder"]["motor"]),
            ),
            JointConfig(
                config["elbow"]["mass"],
                config["elbow"]["length"],
                config["elbow"]["moi"],
                config["elbow"]["cgRadius"],
                self._get_motor(config["elbow"]["motor"]),
            ),
        )

        # Set up time variables
        self._total_time = opti.variable()
        dt = self._total_time / (n + 1)
        opti.subject_to(self._total_time > 0)
        opti.subject_to(self._total_time < 10)
        opti.minimize(self._total_time)

        # Create velocity parameters
        self._initial_velocity_0 = opti.parameter()
        self._initial_velocity_1 = opti.parameter()
        self._final_velocity_0 = opti.parameter()
        self._final_velocity_1 = opti.parameter()

        # Create theta points
        theta_points = []
        self._theta_points = theta_points
        theta_points.append([opti.parameter(), opti.parameter()])
        for _ in range(n):
            theta_0 = opti.variable()
            theta_1 = opti.variable()
            opti.subject_to(
                opti.bounded(
                    config["shoulder"]["minAngle"],
                    theta_0,
                    config["shoulder"]["maxAngle"],
                )
            )
            opti.subject_to(
                opti.bounded(
                    config["elbow"]["minAngle"], theta_1, config["elbow"]["maxAngle"]
                )
            )
            theta_points.append([theta_0, theta_1])
        theta_points.append([opti.parameter(), opti.parameter()])

        # Create constraint parameters (0 = disabled, 1 = enabled)
        self._constraint_parameters = {}
        for constraint in config["constraints"].items():
            self._constraint_parameters[constraint[0]] = opti.parameter()

        # Apply point constraints
        for i in range(n + 2):
            # Get surrounding points
            if i == 0:
                last_theta = [
                    theta_points[i][0] - (self._initial_velocity_0 * dt),
                    theta_points[i][1] - (self._initial_velocity_1 * dt),
                ]
                current_theta = theta_points[i]
                next_theta = theta_points[i + 1]
            elif i == n + 1:
                last_theta = theta_points[i - 1]
                current_theta = theta_points[i]
                next_theta = [
                    theta_points[i][0] + (self._initial_velocity_0 * dt),
                    theta_points[i][1] + (self._initial_velocity_1 * dt),
                ]
            else:
                last_theta = theta_points[i - 1]
                current_theta = theta_points[i]
                next_theta = theta_points[i + 1]

            # Apply voltage constraints
            last_velocity = (
                (current_theta[0] - last_theta[0]) / dt,
                (current_theta[1] - last_theta[1]) / dt,
            )
            next_velocity = (
                (next_theta[0] - current_theta[0]) / dt,
                (next_theta[1] - current_theta[1]) / dt,
            )
            acceleration = (
                (next_velocity[0] - last_velocity[0]) / dt,
                (next_velocity[1] - last_velocity[1]) / dt,
            )
            voltage = ff_model.calculate(current_theta, last_velocity, acceleration)
            opti.subject_to(opti.bounded(-max_voltage, voltage[0], max_voltage))
            opti.subject_to(opti.bounded(-max_voltage, voltage[1], max_voltage))

            # Apply position constraints
            if i != 0 and i != n + 1:
                x = (
                    config["origin"][0]
                    + config["shoulder"]["length"] * cos(current_theta[0])
                    + config["elbow"]["length"]
                    * cos(current_theta[0] + current_theta[1])
                )
                y = (
                    config["origin"][1]
                    + config["shoulder"]["length"] * sin(current_theta[0])
                    + config["elbow"]["length"]
                    * sin(current_theta[0] + current_theta[1])
                )

                opti.subject_to(y > -10)  # Always stay above the ground
                for constraint in config["constraints"].items():
                    enabled = self._constraint_parameters[constraint[0]]
                    type = constraint[1]["type"]
                    args = constraint[1]["args"]
                    if type == "circle":
                        center_x = args[0]
                        center_y = args[1]
                        radius = args[2]
                        opti.subject_to(
                            sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
                            > radius * enabled
                        )
                    elif type == "rectangle":
                        x_0 = args[0]
                        y_0 = args[1]
                        x_1 = args[2]
                        y_1 = args[3]
                        opti.subject_to(
                            fmax(
                                fabs((2 * (x - ((x_0 + x_1) / 2)) / (x_1 - x_0))),
                                fabs((2 * (y - ((y_0 + y_1) / 2)) / (y_1 - y_0))),
                            )
                            > enabled
                        )

    def _get_motor(self, motor_config) -> DCMotor:
        if motor_config["type"] == "neo":
            return DCMotor.getNEO(motor_config["count"]).withReduction(
                motor_config["reduction"]
            )
        elif motor_config["type"] == "neo550":
            return DCMotor.getNEO550(motor_config["count"]).withReduction(
                motor_config["reduction"]
            )

    def solve(self, parameters):
        opti = self._opti
        theta_points = self._theta_points

        # Update position and velocity parameters
        opti.set_value(
            self._initial_velocity_0, parameters["initialJointVelocities"][0]
        )
        opti.set_value(
            self._initial_velocity_1, parameters["initialJointVelocities"][1]
        )
        opti.set_value(self._final_velocity_0, parameters["finalJointVelocities"][0])
        opti.set_value(self._final_velocity_1, parameters["finalJointVelocities"][1])
        opti.set_value(theta_points[0][0], parameters["initialJointPositions"][0])
        opti.set_value(theta_points[0][1], parameters["initialJointPositions"][1])
        opti.set_value(
            theta_points[len(theta_points) - 1][0], parameters["finalJointPositions"][0]
        )
        opti.set_value(
            theta_points[len(theta_points) - 1][1], parameters["finalJointPositions"][1]
        )

        # Set initial values
        opti.set_initial(self._total_time, 1)
        n = len(theta_points) - 2
        for i in range(1, n + 1):
            opti.set_initial(
                theta_points[i][0],
                (
                    parameters["finalJointPositions"][0]
                    - parameters["initialJointPositions"][0]
                )
                * (i / (n + 2))
                + parameters["initialJointPositions"][0],
            )
            opti.set_initial(
                theta_points[i][1],
                (
                    parameters["finalJointPositions"][1]
                    - parameters["initialJointPositions"][1]
                )
                * (i / (n + 2))
                + parameters["initialJointPositions"][1],
            )

        # Set constraint parameters
        for constraint in self._constraint_parameters.items():
            key = constraint[0]
            value = constraint[1]
            if key in parameters["constraintKeys"]:
                opti.set_value(value, 1)
            else:
                opti.set_value(value, 0)

        # Solve
        try:
            opti.solve()
        except:
            return None

        # Get results
        result = (opti.value(self._total_time), [], [])
        for theta in theta_points:
            result[1].append(opti.value(theta[0]))
            result[2].append(opti.value(theta[1]))
        return result
