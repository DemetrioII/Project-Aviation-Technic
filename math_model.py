import numpy as np
import matplotlib.pyplot as plt
import json
from scipy.integrate import odeint

# Параметры
F_t = 10026_000  # Тяга
c = 0.57  # Коэффициент сопротивления
rho = 1.225  # Плотность
g = 9.81  # Ускорение свободного падения
M = 0.028  # Масса
h = 0  # Высота
R = 8.31  # УГП
T = 300  # Температура
V = 0  # Скорость
S = 186.27  # Площадь
GM3 = 6.674 * 10 ** (-11) * 5.2915793e22  # Гравитационная постоянная

R_e = 600_000

# Начальные условия
x1_0 = 0  # Начальное значение x1
x2_0 = 0  # Начальное значение x2
x3_0 = 0  # Начальное значение x3
x4_0 = 0  # Начальное значение x4
x5_0 = 206_923  # Начальное значение x5
x6_0 = x6 = 90  # Начальное значение x6
stages = [{'F_t': 3_800_000, 'mass': 103_000, 'time': 78},
          {'F_t': 770_800, 'mass': 28_000, 'time': 100}]

accelerations = []


def f(data, stage):
    x1, x2, x3, x4, x5, x6 = data
    global T
    fuel = stages[stage]['mass']
    F_t = stages[stage]['F_t']
    t = stages[stage]['time']
    dm = fuel / t
    if T > 30:
        T = 300 - 6 * (x2 // 1000)
    x6 = 90 * (1 - x2 / 80000)
    if x6 <= 0:
        x6 = 0

    V = (x3 ** 2 + x4 ** 2) ** 0.5
    dx3 = (F_t - c * (rho * np.exp(-g * M * x2 / (R * T)) * V ** 2 * S / 2)) / x5 * np.cos(np.radians(x6))
    dx4 = (F_t - c * (rho * np.exp(-g * M * x2 / (R * T)) * V ** 2 * S / 2)) / x5 * np.sin(
        np.radians(x6)) - GM3 / (R_e + x2) ** 2
    dx5 = -dm
    dx6 = -90 * x2 / 80000

    return [x3, x4, dx3, dx4, dx5, dx6]


time_stage_first = np.linspace(0, stages[0]["time"])
odeint_first = odeint(f, [x1_0, x2_0, x3_0, x4_0, x5_0, x6_0], time_stage_first, args=(0,))

time_stage_second = np.linspace(0, stages[1]["time"], 100)
odeint_second = odeint(f, np.concatenate([odeint_first[-1, :4], [odeint_first[-1, 4] - 40_670], [odeint_first[-1][5]]]), time_stage_second, args=(1,))

time = np.concatenate([time_stage_first, time_stage_first[-1] + time_stage_second])
x = np.concatenate([odeint_first[:, 0], odeint_second[:, 0]])
y = np.concatenate([odeint_first[:, 1], odeint_second[:, 1]])

x_speeds = np.concatenate([odeint_first[:, 2], odeint_second[:, 2]])
y_speeds = np.concatenate([odeint_first[:, 3], odeint_second[:, 3]])

accelerations.append(0)
for i in range(1, len(x_speeds)):
    accelerations.append(((x_speeds[i] - x_speeds[i - 1]) ** 2 + (y_speeds[i] - y_speeds[i - 1]) ** 2) ** 0.5)

plt.plot(time, y_speeds, label="Мат.модель")
with open("data_for_ksp.json", 'r', encoding="UTF-8") as file:
    data = json.load(file)

plt.plot(data[0]["pastime"][1:], data[0]["oy_velocity"][1:], label="KSP")
plt.ylabel("скорость, м/с")
plt.xlabel("время, с")
plt.legend()
plt.show()
