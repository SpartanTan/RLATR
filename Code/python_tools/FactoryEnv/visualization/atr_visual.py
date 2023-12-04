from factory_env.robot.ATR_RK4 import ATR_RK4
import numpy as np
import matplotlib.pyplot as plt

from factory_env.envs.parameters import env_param


params = env_param()
atr: ATR_RK4 = ATR_RK4(params.atr_param)
init_state = np.array([0.0, 0.0, np.pi/2])
atr.reset(init_state)
trajectory = []
wheel_speeds = np.array([1.0, 1.6])
for i in range(500):
    if i > 150:
        wheel_speeds = np.array([1.6, 0.8])
    atr.runge_kutta_4_step(wheel_speeds)
    trajectory.append(atr.state)
    plt.scatter(trajectory[i][0], trajectory[i][1], c='b', s=0.5)
    plt.pause(0.1)
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.axis('equal')
plt.show()