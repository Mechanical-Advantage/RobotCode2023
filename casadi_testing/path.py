from casadi import *
import matplotlib.pyplot as plt

opti = Opti()

n = 50
start_x = opti.parameter()
start_y = opti.parameter()
end_x = opti.parameter()
end_y = opti.parameter()

x_values = []
y_values = []
distance = 0
last_x = start_x
last_y = start_y
for i in range(n):
    x = opti.variable()
    y = opti.variable()
    x_values.append(x)
    y_values.append(y)
    distance = fmax(distance, sqrt(((x - last_x) ** 2) + ((y - last_y) ** 2)))
    last_x = x
    last_y = y
distance = fmax(distance, sqrt(((end_x - last_x) ** 2) + ((end_y - last_y) ** 2)))

opti.solver("ipopt", {}, {"max_iter": 5000})
start_point = (0, 0)
end_point = (10, 10)
opti.set_value(start_x, start_point[0])
opti.set_value(start_y, start_point[1])
opti.set_value(end_x, end_point[0])
opti.set_value(end_y, end_point[1])
for i in range(n):
    opti.set_initial(x_values[i], ((end_point[0] - start_point[0]) / (n + 1)) * (i + 1) + start_point[0])
    opti.set_initial(y_values[i], ((end_point[1] - start_point[1]) / (n + 1)) * (i + 1) + start_point[1])
    opti.subject_to(sqrt((x_values[i] - 7) ** 2 + (y_values[i] - 5) ** 2) > 2)
opti.minimize(distance)
try:
    opti.solve()
except:
    pass

x_numbers = [opti.value(start_x)]
y_numbers = [opti.value(start_y)]
# print(opti.value(start_x), opti.value(start_y))
for i in range(n):
    x_numbers.append(opti.value(x_values[i]))
    y_numbers.append(opti.value(y_values[i]))
    # print(opti.value(x_values[i]), opti.value(y_values[i]))
x_numbers.append(opti.value(end_x))
y_numbers.append(opti.value(end_y))
# print(opti.value(end_x), opti.value(end_y))

plt.scatter(x_numbers, y_numbers)
plt.plot(x_numbers, y_numbers)
plt.show()