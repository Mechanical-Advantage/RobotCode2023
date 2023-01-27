# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import time
from math import pi

import matplotlib.animation
import matplotlib.pyplot as plt
from casadi import *

from DCMotor import DCMotor
from DoubleJointedArmFeedforward import DoubleJointedArmFeedforward, JointConfig

opti = Opti()
opti.solver("ipopt", {}, {"max_iter": 500})

n = 25
max_voltage = 8
L = (0.85, 0.75)
ffModel = DoubleJointedArmFeedforward(
    JointConfig(15.0, L[0], 1, L[0] / 2, DCMotor.getNEO(1).withReduction(130.0)),
    JointConfig(10.0, L[1], 1, L[1] / 2, DCMotor.getNEO(1).withReduction(100.0)),
)
start_theta = (opti.parameter(), opti.parameter())
end_theta = (opti.parameter(), opti.parameter())

opti.set_value(start_theta[0], pi / 4)
opti.set_value(start_theta[1], -pi / 2)
opti.set_value(end_theta[0], (2 * pi) / 3)
opti.set_value(end_theta[1], -(5.5 * pi) / 3)
# opti.set_value(start_theta[0], 1.3)
# opti.set_value(start_theta[1], -3)
# opti.set_value(end_theta[0], 0.8)
# opti.set_value(end_theta[1], -0.2)

# Set up time constraints
total_time = opti.variable()
dt = total_time / (n + 2)
opti.subject_to(total_time > 0)
opti.subject_to(total_time < 10)
opti.set_initial(total_time, 1)
opti.minimize(total_time)

# Create theta points
theta_points = []
theta_points.append(start_theta)
for i in range(n):
    theta = (opti.variable(), opti.variable())
    theta_points.append(theta)
    opti.set_initial(
        theta[0],
        (opti.value(end_theta[0]) - opti.value(start_theta[0])) * ((i + 1) / (n + 2))
        + opti.value(start_theta[0]),
    )
    opti.set_initial(
        theta[1],
        (opti.value(end_theta[1]) - opti.value(start_theta[1])) * ((i + 1) / (n + 2))
        + opti.value(start_theta[1]),
    )
theta_points.append(end_theta)

# Apply point constraints
voltages = []
for i in range(n + 2):
    last_theta = start_theta if i == 0 else theta_points[i - 1]
    current_theta = theta_points[i]
    next_theta = end_theta if i == len(theta_points) - 1 else theta_points[i + 1]

    # Voltage constraints
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
    voltage = ffModel.calculate(current_theta, last_velocity, acceleration)
    voltages.append(voltage)
    opti.subject_to(opti.bounded(-max_voltage, voltage[0], max_voltage))
    opti.subject_to(opti.bounded(-max_voltage, voltage[1], max_voltage))

    # Position constraints
    if i != 0 and i != len(theta_points) - 1:
        wrist_base = (
            L[0] * cos(current_theta[0])
            + L[1] * cos(current_theta[0] + current_theta[1]),
            L[0] * sin(current_theta[0])
            + L[1] * sin(current_theta[0] + current_theta[1]),
        )
        opti.subject_to(wrist_base[1] > 0)

        opti.subject_to(
            sqrt((wrist_base[0] + 0.5) ** 2 + (wrist_base[1] - 0.25) ** 2) > 0.35
        )

        # print(if_else(-0.75 < wrist_base[0] < -0.25, wrist_base[1], 1))
        # opti.subject_to(if_else(-0.75 < wrist_base[0] < -0.25, wrist_base[1], 1) > 0.5)

# Solve
# start_time = time.time()
# for i in range(100):
#     opti.solve()
# end_time = time.time()
# print((end_time - start_time) / 100)
# sys.exit(0)
try:
    opti.solve()
except:
    pass


# Show plot
[print(opti.value(x[0]), opti.value(x[1])) for x in voltages]
print("Total time =", opti.value(total_time))
print("DT =", opti.value(dt))

animation_points = []
current_time = 0
for i in range(n + 2):
    animation_points.append(
        (current_time, opti.value(theta_points[i][0]), opti.value(theta_points[i][1]))
    )
    current_time += opti.value(dt)

fig, ax = plt.subplots()
circle = plt.Circle((-0.5, 0.25), 0.25)
start_time = time.time()


def animate(i):
    current_time = (time.time() - start_time) % (opti.value(total_time) + 2) - 1
    next_index = 0
    while (
        next_index < len(animation_points)
        and animation_points[next_index][0] < current_time
    ):
        next_index += 1
    if next_index >= len(animation_points):
        next_index = len(animation_points) - 1
    last_point = animation_points[next_index - 1]
    next_point = animation_points[next_index]

    t = (current_time - last_point[0]) / (next_point[0] - last_point[0])
    t = 1 if t > 1 else t
    t = 0 if t < 0 else t
    theta_1 = (next_point[1] - last_point[1]) * t + last_point[1]
    theta_2 = (next_point[2] - last_point[2]) * t + last_point[2]

    x = [
        0,
        L[0] * cos(theta_1),
        L[0] * cos(theta_1) + L[1] * cos(theta_1 + theta_2),
    ]
    y = [
        0,
        L[0] * sin(theta_1),
        L[0] * sin(theta_1) + L[1] * sin(theta_1 + theta_2),
    ]
    ax.clear()
    ax.plot([-2, 2], [0, 0])
    ax.add_patch(circle)
    # ax.plot([-0.75, -0.75, -0.25, -0.25, -0.75], [0.5, 0, 0, 0.5, 0.5])
    ax.plot(x, y)

    ax.set_xlim([-2, 2])
    ax.set_ylim([-0.5, 2.6])


animation = matplotlib.animation.FuncAnimation(fig, animate, interval=15)

plt.show()
