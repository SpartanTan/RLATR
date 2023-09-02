import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
wheel_radius = 0.125
track_width = 0.48

# Function to compute the robot's pose given its current state and wheel speeds


def robot_pose_derivative(state, wheel_speeds):
    x, y, theta = state
    left_wheel_speed, right_wheel_speed = wheel_speeds

    # Compute the linear and angular velocities
    linear_velocity = wheel_radius * (left_wheel_speed + right_wheel_speed) / 2
    angular_velocity = wheel_radius * \
        (right_wheel_speed - left_wheel_speed) / track_width

    # Calculate the change in x, y, and theta
    dx = linear_velocity * np.cos(theta)
    dy = linear_velocity * np.sin(theta)
    dtheta = angular_velocity

    return np.array([dx, dy, dtheta])

# Runge-Kutta 4 method to update the robot's pose


def runge_kutta_4_step(state, wheel_speeds, dt):
    k1 = robot_pose_derivative(state, wheel_speeds)
    k2 = robot_pose_derivative(state + dt / 2 * k1, wheel_speeds)
    k3 = robot_pose_derivative(state + dt / 2 * k2, wheel_speeds)
    k4 = robot_pose_derivative(state + dt * k3, wheel_speeds)

    new_state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return new_state


def switch_wheel_speeds(wheel_speeds):
    """
    Input wheel_speed: [wr, wl]
    Output wheel_speed: [wl, wr]
    """
    return [wheel_speeds[1], wheel_speeds[0]]

# Simulation parameters
simulation_duration = 10  # in seconds
time_step = 0.1  # in seconds
wheel_speeds = (2.0, 4.0)  # in rad/s (right wheel, left wheel)

wheel_speeds = switch_wheel_speeds(wheel_speeds)

# Initial state (x, y, theta)
state = np.array([0, 0, 0], dtype=float)
# Store the robot's trajectory
trajectory = [state[:2]]

# Simulate the robot's movement
n_steps = int(simulation_duration / time_step)
for step in range(n_steps):
    state = runge_kutta_4_step(state, wheel_speeds, time_step)
    trajectory.append(state[:2])

# Convert the trajectory list to a NumPy array
trajectory = np.array(trajectory)
# Plot the trajectory
plt.plot(trajectory[:, 0], trajectory[:, 1], marker='o',
         markersize=2, linestyle='-', linewidth=1)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Trajectory')
plt.grid(True)
plt.axis('equal')

plt.scatter(trajectory[-1, 0], trajectory[-1, 1], color='red')
plt.text(trajectory[-1, 0]-2, trajectory[-1, 1]+0.2,
         f'{trajectory[-1,0], trajectory[-1, 1]}')
plt.show()
