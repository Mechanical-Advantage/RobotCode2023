from casadi import *
import math

opti = Opti()
opti.solver("ipopt", {}, {"max_iter": 50})

L_1 = 3
L_2 = 2

target_x = opti.parameter()
target_y = opti.parameter()

theta_1 = opti.variable()
theta_2 = opti.variable()
opti.subject_to(opti.bounded(0, theta_1, math.pi))
opti.subject_to(opti.bounded(-math.pi, theta_2, math.pi))
opti.set_initial(theta_1, math.pi / 2)
opti.set_initial(theta_2, 0)

x = L_1 * cos(theta_1) + L_2 * cos(theta_1 + theta_2)
y = L_1 * sin(theta_1) + L_2 * sin(theta_1 + theta_2)

opti.set_value(target_x, 1)
opti.set_value(target_y, 1)
opti.minimize(sqrt((x - target_x) ** 2 + (y - target_y) ** 2))
try:
    opti.solve()
except:
    pass

print(opti.value(theta_1))
print(opti.value(theta_2))