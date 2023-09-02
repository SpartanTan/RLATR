import gymnasium as gym
import pkg_resources
import numpy as np
from factory_env.envs.parameters import env_param
from factory_env.robot.RF import RangeFinder
import matplotlib.pyplot as plt
import time
from shapely.geometry import LinearRing, LineString, Point
from numba import jit, njit

import keyboard
import threading
import multiprocessing
import matplotlib


def print_param(params: env_param):
    print(f"Environment setup: \n\
            Path length: {params.path_param.Lp} m\n\
            Number of obstacles: {params.path_param.Nw}\n\
            ATR max velocity: {params.atr_param.atr_linear_vel_max} m/s\n\
            ATR max wheel speed: {params.atr_param.atr_w_max} rad/s\n\
            target finishing time: {params.path_param.target_finishing_time} s\n\
            step time: {params.atr_param.dt} s\n\
            ")
    
def check_param(params: env_param):
    print_param(params)
    assert params.path_param.target_finishing_time >= params.path_param.Lp / params.atr_param.atr_linear_vel_max, "The target finishing time is too short."

def draw_arrow(ax, x, y, angle, length=1, color='b', alpha=1.0):
    dx = length * np.cos(angle)
    dy = length * np.sin(angle)
    ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc=color, ec='black', alpha=alpha)
                                      

def calculate_all_intersections_vectorized_optimized(obstacles, sensor_angles, max_range=1.0):
    x1, y1 = env.atr.state[0], env.atr.state[1]  # Assuming sensor is at the origin of the car frame

    # Pre-compute A for all angles
    A_values = np.cos(sensor_angles)**2 + np.sin(sensor_angles)**2

    intersections = []
    measurements = []
    for i, theta in enumerate(sensor_angles):
        A = A_values[i]  # Fetch pre-computed A value
        intersection_found = False

        intersections_temp = []  # Temp list for intersections
        for obstacle in obstacles:
            # Calculate B, C, and discriminant for each obstacle
            B = 2 * (np.cos(theta) * (x1 - obstacle[0]) + np.sin(theta) * (y1 - obstacle[1]))
            C = (x1 - obstacle[0])**2 + (y1 - obstacle[1])**2 - obstacle[2]**2
            discriminant = B**2 - 4*A*C

            if discriminant >= 0:  # Intersection exists
                # Calculate both intersections
                t1 = (-B + np.sqrt(discriminant)) / (2*A)
                x2 = x1 + t1 * np.cos(theta)
                y2 = y1 + t1 * np.sin(theta)
                t2 = (-B - np.sqrt(discriminant)) / (2*A)
                x3 = x1 + t2 * np.cos(theta)
                y3 = y1 + t2 * np.sin(theta)
                intersections_temp += [(x2, y2, t1), (x3, y3, t2)]

        # Sort by t value so that we first check the intersection closer to the sensor
        intersections_temp.sort(key=lambda x: x[2])

        for (x, y, t) in intersections_temp:
            # Check if the intersection is in the direction of the beam and within max range
            if theta >= -np.pi/2 and theta <= np.pi/2:  # Forward direction is positive x
                if x >= x1 and t <= max_range:
                    intersections.append((x, y))
                    measurements.append((theta, t))
                    intersection_found = True
                    break
            else:  # Forward direction is negative x
                if x <= x1 and t <= max_range:
                    intersections.append((x, y))
                    measurements.append((theta, t))
                    intersection_found = True
                    break

        if not intersection_found:
            measurements.append((theta, 1.0))
            
    return intersections, measurements


def all_intersections(robot_pos, obstacles, sensor_angles, walls, max_range=1.0):
    """
    This function calculates all intersections between the sensor beams and the obstacles and the walls.
    """
    x1, y1 = robot_pos[0], robot_pos[1]  # Assuming sensor is at the origin of the car frame

    # Pre-compute A for all angles
    A_values = np.cos(sensor_angles)**2 + np.sin(sensor_angles)**2

    intersections = []
    measurements = []
    for i, theta in enumerate(sensor_angles):
        A = A_values[i]  # Fetch pre-computed A value
        intersection_found = False

        intersections_temp = []  # Temp list for intersections
        for obstacle in obstacles:
            # Calculate B, C, and discriminant for each obstacle
            B = 2 * (np.cos(theta) * (x1 - obstacle[0]) + np.sin(theta) * (y1 - obstacle[1]))
            C = (x1 - obstacle[0])**2 + (y1 - obstacle[1])**2 - obstacle[2]**2
            discriminant = B**2 - 4*A*C

            if discriminant >= 0:  # Intersection exists
                # Calculate both intersections
                t1 = (-B + np.sqrt(discriminant)) / (2*A)
                x2 = x1 + t1 * np.cos(theta)
                y2 = y1 + t1 * np.sin(theta)
                t2 = (-B - np.sqrt(discriminant)) / (2*A)
                x3 = x1 + t2 * np.cos(theta)
                y3 = y1 + t2 * np.sin(theta)
                intersections_temp += [(x2, y2, t1), (x3, y3, t2)]
        
        ## Calculate intersection with walls
        x_end = x1 + max_range * np.cos(theta)
        y_end = y1 + max_range * np.sin(theta)
        beam = LineString([(x1, y1), (x_end, y_end)])
        intersection = walls.intersection(beam)
        
        if intersection.is_empty:
            pass  # No intersection
        elif intersection.geom_type == 'Point':
                t_wall = ((intersection.x - x1)**2 + (intersection.y - y1)**2)**0.5
                intersections_temp.append((intersection.x, intersection.y, t_wall))
        elif intersection.geom_type == 'MultiPoint':
            for point in intersection.geoms:
                t_wall = ((point.x - x1)**2 + (point.y - y1)**2)**0.5
                intersections_temp.append((point.x, point.y, t_wall))
                
        # Sort by t value so that we first check the intersection closer to the sensor
        intersections_temp.sort(key=lambda x: x[2])

        for (x, y, t) in intersections_temp:
            # Check if the intersection is in the direction of the beam and within max range
            if theta >= -np.pi/2 and theta <= np.pi/2:  # Forward direction is positive x
                if x >= x1 and t <= max_range:
                    intersections.append((x, y))
                    measurements.append((theta, t))
                    intersection_found = True
                    break
            else:  # Forward direction is negative x
                if x <= x1 and t <= max_range:
                    intersections.append((x, y))
                    measurements.append((theta, t))
                    intersection_found = True
                    break

        if not intersection_found:
            measurements.append((theta, 1.0))
            
    return intersections, measurements

def feasibility_pooling(x):
    """fastest"""
    # sort x in ascending order
    sorted_x = np.sort(x)

    # for each value xi in sorted x
    for xi in sorted_x:
        di = theta * xi  # calculate arc-length
        y = di / 2  # calculate opening-width
        si = False  # initialize opening found flag

        # for each value xj in x
        for xj in x:
            if xj > xi:
                y += di
                if y > W:
                    si = True
                    break
                else:
                    y += di / 2
                    if y > W:
                        si = True
                        break
                y = 0

        # if si is False for all xj in x, return xi
        if not si:
            return xi

    # return None if no feasible solution found
    return None
            
options = {'init_type': 'load', 'file_name': 'test.pkl'}
# options = {'init_type': 'run', 'file_name': 'test.pkl'}

params:env_param = env_param()
params.atr_param.atr_linear_vel_max = 0.2
params.path_param.target_finishing_time = 30
check_param(params)

env = gym.make('training-factory-v0', params=params,)
next_obs, info = env.reset(options=options)

action = env.action_space.sample()
observation, reward, done, truncated, info = env.step(action)

# print(env.path.obstacles_np)
# print(env.atr.state)

# fig, ax = plt.subplots()
min_bound = -np.deg2rad(120)
max_bound = np.deg2rad(120)
r = np.deg2rad(6) # resolution

sensor_angles = np.linspace(min_bound, max_bound, int(np.rint(abs((max_bound - min_bound) / r)) +1))
# sensor_angles = np.linspace(min_bound, max_bound, int(np.rint(abs((max_bound - min_bound) / r))))

sensor_angles = np.round(sensor_angles, 3) + env.atr.state[2] # sensor angles are now align with the robot
num_sensors = sensor_angles.shape[0]
print(f"number of sensors: {num_sensors}")

slopes = np.tan(sensor_angles)
slopes = np.round(slopes, 3)
# np.where(slopes > 1000.0, 1000.0, slopes)
which_side  = sensor_angles >= 0
# print(np.rad2deg(sensor_angles))
# print(slopes)
# for _, angle in enumerate(sensor_angles):
#     draw_arrow(ax, 0, 0, angle, 1, 'b', 1.0) 
# plt.axis('equal')
# print(np.rad2deg(sensor_angles))

H = np.array([[np.cos(env.atr.state[2]), -np.sin(env.atr.state[2]), 0, 1],
             [np.sin(env.atr.state[2]), np.cos(env.atr.state[2]), 0, 1],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])
inv_H = np.linalg.inv(H)
obstacles = env.path.obstacles_np.copy()
# Add a row of ones for homogeneous coordinates transformation
homogeneous_obstacles = np.concatenate((obstacles, np.ones((obstacles.shape[0], 1))), axis=1)

# Your rotational matrix
H = np.array([[np.cos(env.atr.state[2]), -np.sin(env.atr.state[2]), 0, env.atr.state[0]],
              [np.sin(env.atr.state[2]), np.cos(env.atr.state[2]), 0, env.atr.state[1]],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
inv_H = np.linalg.inv(H)

# transform the coordinates
transformed_obstacles_homogeneous = np.dot(inv_H, homogeneous_obstacles.T).T

# Remove the homogeneous coordinate before storing back to the array
transformed_obstacles = transformed_obstacles_homogeneous[:, 0:3]
# transformed_obstacles = transformed_obstacles.tolist()
# print(transformed_obstacles)

# intersections_ = calculate_all_intersections_slope_optimized(transformed_obstacles, slopes)
# intersections = workable(transformed_obstacles, slopes)


# start_time = time.time()
# for i in range(1000):
#     intersections_ = calculate_all_intersections_slope_optimized(transformed_obstacles.copy(), slopes)
# print(f"end time: {time.time() - start_time}")
exterior =  np.vstack([env.path.start_line, env.path.wall_up, env.path.end_line[::-1], env.path.wall_down[::-1]])
ring = LinearRing(exterior)

# intersections_, measurements = all_intersections(obstacles.copy(), sensor_angles, ring)
# print(len(intersections_))
rf = RangeFinder(params.sensor_param, params.atr_param.track_width)
rf.reset(ring)

start_time = time.time()
for i in range(300):
    # intersections_, measurements = all_intersections(env.atr.state[0:2], obstacles.copy(), sensor_angles, ring)
    # intersections_, measurements = all_intersections_numba(env.atr.state[0:2], obstacles.copy(), sensor_angles)
    intersections_, measurements, fb_results, closeness = rf.closeness_cal(env.atr.state, obstacles)

print(f"end time for 300 times: {time.time() - start_time}")
# for i in intersections_:
#     print(i)
# raise("stop")
measurements = np.array(measurements)
# print(f"length of measurements: {len(measurements)}")

# raise ("stop")
# start_time = time.time()
# for i in range(1000):
#     intersections_, measurements = calculate_all_intersections_vectorized_optimized(obstacles.copy(), sensor_angles)
# print(f"end time: {time.time() - start_time}")

fig, ax2 = plt.subplots()
for i, data in enumerate(measurements):
    draw_arrow(ax2, 0, 0, data[0], data[1], 'b', 1.0)
plt.axis('equal')

# start_time = time.time()
# for i in range(1000):
#     intersections_ = calculate_all_intersections_slope_optimized_ele(transformed_obstacles.copy(), slopes)
# print(f"end time: {time.time() - start_time}")

# start_time = time.time()
# for i in range(1000):
#     intersections_ = new(transformed_obstacles.copy(), slopes)
# print(f"end time: {time.time() - start_time}")



# print(intersections)
fig, ax = plt.subplots()
for idx, obs in enumerate(obstacles):
    # p_obst, r_obst = obs
    p_obst = obs[0:2]
    r_obst = obs[2]
    circle = plt.Circle((p_obst[0], p_obst[1]), r_obst, color='r', alpha=0.5)
    
    ax.add_patch(circle)
    ax.text(p_obst[0], p_obst[1], idx+1, fontsize=12)

for angle in sensor_angles:
    # angle += env.atr.state[2]
    draw_arrow(ax, env.atr.state[0], env.atr.state[1], angle, 1.0)
plt.scatter([x for x, y in intersections_], [y for x, y in intersections_], color='g', alpha=0.5)
plt.scatter(env.path.wall_up[:, 0], env.path.wall_up[:, 1], s=2, marker='o', rasterized=True)
plt.scatter(env.path.wall_down[:, 0], env.path.wall_down[:, 1], s=2, marker='o', rasterized=True)
plt.axis('equal')

angles, distances = zip(*measurements)
mes = np.array(distances)

first_distance = mes[0]
rest_distances = mes[1:].reshape(20, -1)
rest_distances_list = rest_distances.tolist()
rest_distances_list[0].insert(0, first_distance)
mes = rest_distances_list

# mes = np.array(measurements)
# mes = mes[0:, 1] # this line for extracting all distances

# print(f"measurement shape before reshape: {mes.shape}")

# This step is to redistribute the sensors in the same sector to be in one row
# 20 sectors, 240 degrees, 12 degrees covered by each sector
# resolution is 6 degrees,
# mes = mes.reshape((20, -1))

# print(mes)
# print(f"measurement shape after reshape: {mes.shape}")

# raise("stop")
W = 0.48
N = sensor_angles.shape[0]
Ss = np.deg2rad(240)
# print(span)
theta = Ss / (N - 1)

results = []
for i, x in enumerate(mes):
    results.append(feasibility_pooling(x))

results = fb_results
fig, ax3 = plt.subplots()
for idx, angle in enumerate(params.sensor_param.angles_of_sectors):
    angle += env.atr.state[2]
    draw_arrow(ax3, 0, 0, angle, results[idx], 'r', 1.0)
for angle in params.sensor_param.sectors:
    angle += env.atr.state[2]
    draw_arrow(ax3, 0, 0, angle, 1.0)
plt.axis('equal')
env.render()

plt.show()
