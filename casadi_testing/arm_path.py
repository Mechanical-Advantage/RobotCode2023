from casadi import *
from math import pi
import time
import matplotlib.pyplot as plt
import matplotlib.animation
import sys

opti = Opti()
opti.solver("ipopt")

n = 25
L_1 = 3
L_2 = 2.5
theta_1_max_v = 2
theta_2_max_v = 2
theta_1_max_a = 2
theta_2_max_a = 2
start_theta_1 = opti.parameter()
start_theta_2 = opti.parameter()
end_theta_1 = opti.parameter()
end_theta_2 = opti.parameter()

# opti.set_value(start_theta_1, pi / 3 - 0.15)
# opti.set_value(start_theta_2, -(2 * pi) / 3)
# opti.set_value(end_theta_1, (3 * pi) / 4)
# opti.set_value(end_theta_2, -2.5 * pi)
opti.set_value(start_theta_1, 1.3)
opti.set_value(start_theta_2, -3)
opti.set_value(end_theta_1, 0.8)
opti.set_value(end_theta_2, -0.2)

theta_1_points = []
theta_2_points = []

total_time = opti.variable()
dt = total_time / (n + 2)
opti.subject_to(total_time > 0)
opti.subject_to(total_time < 30)
opti.set_initial(total_time, 10)

# Create theta points
theta_1_points.append(start_theta_1)
theta_2_points.append(start_theta_2)
for i in range(n):
    theta_1 = opti.variable()
    theta_2 = opti.variable()
    theta_1_points.append(theta_1)
    theta_2_points.append(theta_2)
theta_1_points.append(end_theta_1)
theta_2_points.append(end_theta_2)

# Apply position constraints
for i in range(n):
    theta_1 = theta_1_points[i + 1]
    theta_2 = theta_2_points[i + 1]
    x = L_1 * cos(theta_1) + L_2 * cos(theta_1 + theta_2)
    y = L_1 * sin(theta_1) + L_2 * sin(theta_1 + theta_2)
    opti.subject_to(y > 0)
    opti.subject_to(y > x * 0.8 - 0.5)
    opti.subject_to(x > -2)

# Apply velocity constraints
for i in range(n + 1):
    theta_1 = theta_1_points[i]
    theta_2 = theta_2_points[i]
    next_theta_1 = theta_1_points[i + 1]
    next_theta_2 = theta_2_points[i + 1]
    theta_1_v = (next_theta_1 - theta_1) / dt
    theta_2_v = (next_theta_2 - theta_2) / dt
    opti.subject_to(opti.bounded(-theta_1_max_v, theta_1_v, theta_1_max_v))
    opti.subject_to(opti.bounded(-theta_2_max_v, theta_2_v, theta_2_max_v))

# Apply acceleration constraints
for i in range(n + 2):
    last_theta_1 = start_theta_1 if i == 0 else theta_1_points[i - 1]
    last_theta_2 = start_theta_2 if i == 0 else theta_2_points[i - 1]
    theta_1 = theta_1_points[i]
    theta_2 = theta_2_points[i]
    next_theta_1 = end_theta_1 if i == len(theta_1_points) - 1 else theta_1_points[i + 1]
    next_theta_2 = end_theta_2 if i == len(theta_1_points) - 1 else theta_2_points[i + 1]
    theta_1_a = (((next_theta_1 - theta_1) / dt) - ((theta_1 - last_theta_1) / dt)) / dt
    theta_2_a = (((next_theta_2 - theta_2) / dt) - ((theta_2 - last_theta_2) / dt)) / dt
    opti.subject_to(opti.bounded(-theta_1_max_a, theta_1_a, theta_1_max_a))
    opti.subject_to(opti.bounded(-theta_2_max_a, theta_2_a, theta_2_max_a))
    

# Solve
opti.minimize(total_time)

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

# for i in range(n + 1):
#     print(opti.value(theta_1_points[i]), opti.value(theta_2_points[i]))
# print(opti.value(end_theta_1), opti.value(end_theta_2))
print("Total time =", opti.value(total_time))
print("DT =", opti.value(dt))

animation_points = []
current_time = 0
for i in range(n + 1):
    animation_points.append((current_time, opti.value(theta_1_points[i]), opti.value(theta_2_points[i])))
    current_time += opti.value(dt)
animation_points.append((current_time, opti.value(end_theta_1), opti.value(end_theta_2)))

fig, ax = plt.subplots()
start_time = time.time() + 1
def animate(i):
    current_time = time.time() - start_time
    next_index = 0
    while next_index < len(animation_points) and animation_points[next_index][0] < current_time:
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
    
    x = [0, L_1 * cos(theta_1), L_1 * cos(theta_1) + L_2 * cos(theta_1 + theta_2)]
    y = [0, L_1 * sin(theta_1), L_1 * sin(theta_1) + L_2 * sin(theta_1 + theta_2)]
    ax.clear()
    ax.plot(x, y)
    ax.plot([-6, 6], [0, 0])
    ax.plot([0, 5], [-0.5, 3.5])
    ax.set_xlim([-6, 6])
    ax.set_ylim([-2, 8])

animation = matplotlib.animation.FuncAnimation(fig, animate, interval=15)

plt.show()