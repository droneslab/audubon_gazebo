#! /usr/bin/env python3

# This gives a purely kinematic solution for the circle test
# Real life performance will differ based on the dynamics
# Only trust this on lower speeds - instead just match the performance of the physical car

from math import sin
from math import cos
from math import tan
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp


plt.style.use('seaborn-poster')

phi = 0.35
speed = 3
wheel_base = 0.175
wheel_radius = 0.052


F1 = lambda t, s1: speed*cos(speed*tan(phi)*t/wheel_base)
F2 = lambda t, s2: speed*sin(speed*tan(phi)*t/wheel_base)

t_eval = np.arange(0, 10.01, 0.01)
sol1 = solve_ivp(F1, [0, 10], [0, 0], t_eval=t_eval)
sol2 = solve_ivp(F2, [0, 10], [0, 0], t_eval=t_eval)


plt.figure(figsize = (8, 8))
plt.plot(sol1.y[0], sol2.y[0])
plt.xlabel('x')
plt.ylabel('y')
plt.show()