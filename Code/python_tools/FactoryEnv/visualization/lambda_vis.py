import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# min_distance_index = np.argmin(self.obstacles_obs[:, 0])
min_distance = np.linspace(0, 1, 100)
corresponding_angle = np.linspace(0, 3*np.pi/4, 100)
gamma_lambda = 0.8
# Lambda = np.tanh(gamma_lambda * ((min_distance+1) * (np.sin(min(corresponding_angle, np.pi/2))+1)-1))
Lambda = np.tanh(gamma_lambda * ((min_distance + 1) * (np.sin(np.minimum(corresponding_angle, np.pi/2)) + 1) - 1))
# Creating a meshgrid for plotting
min_distance_grid, corresponding_angle_grid = np.meshgrid(min_distance, corresponding_angle)
Lambda_grid = np.tanh(gamma_lambda * ((min_distance_grid + 1) * 
                  (np.sin(np.minimum(np.abs(corresponding_angle_grid), np.pi / 2)) + 1) - 1))
# Creating a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(min_distance_grid, corresponding_angle_grid, Lambda_grid, cmap=plt.cm.viridis, linewidth=0, antialiased=False)
ax.set_xlabel('Minimum Distance')
ax.set_ylabel('Corresponding Angle')
ax.set_zlabel('Lambda')
ax.set_title('Lambda as a function of Minimum Distance and Corresponding Angle')
plt.show()