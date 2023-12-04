import argparse
import numpy as np
from matplotlib import pyplot as plt
# Import the Path class from your env.py file
from factory_env.envs.parameters import env_param, path_param, reward_param, ATR_param, sensor_param
from factory_env.map.env_path import Path
import pickle
import time

# from .LQR import lqr_control, mpc_control
from Astar import AStar
from LQR import lqr_control, mpc_control
from factory_env.robot.ATR_RK4 import ATR_RK4

def strtobool(val):
    """
    Convert a string representation of truth to true (1) or false (0).
    
    True values are 'y', 'yes', 't', 'true', 'on', and '1'.
    False values are 'n', 'no', 'f', 'false', 'off', and '0'.
    Raises ValueError if 'val' is anything else.
    """
    val = val.lower()
    if val in ('y', 'yes', 't', 'true', 'on', '1'):
        return 1
    elif val in ('n', 'no', 'f', 'false', 'off', '0'):
        return 0
    else:
        raise ValueError("Invalid truth value %r" % (val,))
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dynamic', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='if toggled, cuda will not be enabled by default')
    args = parser.parse_args()
    return args

def draw_arrow(x, y, angle, length=0.5, color='b'):
    dx = length * np.cos(angle)
    dy = length * np.sin(angle)
    plt.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc=color, ec='black')

def calculate_yaw_angles(trajectory):
    # Initialize an array for yaw angles
    yaw_angles = np.zeros(len(trajectory))
    # Compute yaw angle for each point
    for i in range(len(trajectory) - 1):
        dx = trajectory[i+1, 0] - trajectory[i, 0]
        dy = trajectory[i+1, 1] - trajectory[i, 1]
        yaw_angles[i] = np.arctan2(dy, dx)

    # For the last point, just use the same angle as the second-to-last point
    yaw_angles[-1] = yaw_angles[-2]
    return yaw_angles

def find_target_point(trajectory, point, shift_points):
    # Calculate the squared Euclidean distance to each point in the trajectory
    distances = np.sum((trajectory - point) ** 2, axis=1)

    # Find the index of the closest point
    closest_idx = np.argmin(distances)
    target_idx = closest_idx + shift_points
    if target_idx >= len(trajectory):
        target_idx = len(trajectory) - 1
    # Return the closest point and its index
    target_point = trajectory[target_idx]
    return target_point, target_idx

def generate_half_circle_trajectory(radius, length, num_points=60):
    """
    Generate a half-circle trajectory with a specified radius, total length, and number of points.
    Returns the trajectory as a 2D numpy array with columns for x and y coordinates.
    """
    # Calculate the circumference of the full circle
    circumference = 2 * np.pi * radius

    # Ensure that the requested length does not exceed half of the circumference
    if length > circumference / 2:
        raise ValueError("Requested length exceeds half of the circle's circumference")

    # Calculate the angle corresponding to the requested length
    theta = length / radius

    # Generate points for the half circle
    angles = np.linspace(0, theta, num_points)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)

    # Combine x and y into a 2D numpy array
    trajectory = np.column_stack((x, y))
    return trajectory

if __name__ == "__main__":
    args = parse_args()
    fig, ax = plt.subplots()
    # Initialize the environment
    params = env_param()
    params.path_param.consider_width = True
    params.path_param.No = 3
    params.path_param.sigma_d = 0.2
    testPath = Path(params.path_param, params.atr_param.atr_linear_vel_max)
    testPath.reset()
    # testPath = pickle.load(open("path_instance.pkl", "rb"))
    # testPath.reset()

    
    init_state = np.array([testPath.waypoints[0][0], testPath.waypoints[0][1], testPath.yaw_angles[0]])
    
    # Define the system parameters
    R = 0.125 # radius of the wheels
    L = 0.48 # distance between the wheels
    Delta_t = 0.1 # sampling time
    atr = ATR_RK4(params.atr_param)
    atr.reset(init_state)

    # Define the weighting matrices Q and R
    Q = np.diag([1, 1, 0.1])  # State weighting matrix
    Qf = np.diag([5, 5, 0])  # Final state weighting matrix
    R_input = np.diag([0.01, 0.01])  # Control input weighting matrix

    u_min = np.array([-1.6, -1.6])    # Minimum control input
    u_max = np.array([1.6, 1.6])    # Maximum control input
    N = 10

    # Initialize the DStar planner
    dstar = AStar(testPath, init_state[0:2])
    path = dstar.find_path(len(testPath.even_trajectory) - 1)
    path_yaw = calculate_yaw_angles(path)
    init_state = np.array([path[0][0], path[0][1], path_yaw[0]])
    atr.reset(init_state)

    u = np.array([0, 0])
    target_point, target_idx = find_target_point(path, atr.state[0:2], 5)
    x = None
    traj = np.array([atr.state[0], atr.state[1]])
    for id in range(1000):
        plt.cla()
        # fk_state = testPath.even_trajectory[id+1]
        # fk_yaw = testPath.yaw_angles[id+1]
        # atr.state = np.array([fk_state[0], fk_state[1], fk_yaw])
        # dstar.update_environment(atr.state[0:2])
        # n_points = len(testPath.even_trajectory) - id
        # path = dstar.re_run_pathfinding(n_points)
        is_arrived = testPath.is_arrived(atr.state[0:2], 2)
        target_point, target_idx = find_target_point(path, atr.state[0:2], 3)
        x_ref = np.array([target_point[0], target_point[1], path_yaw[target_idx]])
        # u = lqr_control(atr.state, x_ref, R, L, Delta_t, Q, R_input, u)
        u, x = mpc_control(atr.state, x_ref, R, L, Delta_t, Q, Qf, R_input, u_min, u_max, N, u)
        # clip u
        # u = np.clip(u, u_min, u_max)
        # print(u)
        atr.runge_kutta_4_step(u, "simple")
        traj = np.vstack((traj, atr.state[0:2]))
        draw_arrow(atr.state[0], atr.state[1], atr.state[2])
        
        dstar.plot_path(ax)
        # plt.scatter(path[:, 0], path[:, 1], color='blue', alpha=0.5)
        ax.scatter(target_point[0], target_point[1],
                        s=100, marker='*', label='Target Point')
        if x is not None:
            ax.scatter(x[0, :], x[1, :], color='red', s=0.2, alpha=0.5)
        ax.add_patch(plt.Circle((atr.state[0], atr.state[1]), L/2, color='r', fill=False))
        ax.set_aspect('equal')
        plt.pause(0.2)
        if is_arrived:
            ax.plot(traj[:, 0], traj[:, 1], 'r--')
            plt.show(block=True)
    
    