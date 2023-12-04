import argparse
import numpy as np
from matplotlib import pyplot as plt
# Import the Path class from your env.py file
from factory_env.envs.parameters import env_param, path_param, reward_param, ATR_param, sensor_param
from factory_env.map.env_path import Path
import pickle
import time
from scipy.interpolate import CubicSpline
from scipy.interpolate import UnivariateSpline


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

class AStar:
    def __init__(self, path_instance, init_position, resolution=0.05):
        self.path_instance = path_instance
        self.reference_path = path_instance.even_trajectory
        self.resolution = resolution
        self.current_position = init_position
        self.path_smoothed = np.array(np.zeros((1,2)))

    def update_environment(self, current_position):
        # Update the environment based on the new obstacle positions
        self.path_instance.update_obstacles()
        self.obstacles_np = self.path_instance.obstacles_np
        self.current_position = current_position

    def calculate_offsets(self):
        min_x = min(self.path_instance.waypoints[0,0], self.path_instance.waypoints[-1,0], min(self.path_instance.obstacles_np[:, 0]))
        min_y = min(self.path_instance.waypoints[0,1], self.path_instance.waypoints[-1,1], min(self.path_instance.obstacles_np[:, 1]))
        return -min_x, -min_y
    
    def calculate_grid_costs(self):
        costs = {}
        max_cost = 10  # Define a maximum cost for grid cells far from the reference path
        for x in range(self.grid_size[0]):
            for y in range(self.grid_size[1]):
                grid_point = (x, y)
                real_point = self.to_real_coordinates(grid_point)
                distance = min(np.linalg.norm(np.array(real_point) - np.array(ref_point)) for ref_point in self.reference_path)
                # Assign a higher cost for being further from the reference path
                costs[grid_point] = min(max_cost, distance)  # Cap the cost at max_cost
        return costs
    
    def to_grid_coordinates(self, point):
        normalized_x = (point[0] + self.x_offset) / self.resolution
        normalized_y = (point[1] + self.y_offset) / self.resolution
        return int(normalized_x), int(normalized_y)
    
    def to_real_coordinates(self, grid_point):
        real_x = (grid_point[0] * self.resolution) - self.x_offset
        real_y = (grid_point[1] * self.resolution) - self.y_offset
        return real_x, real_y
    
    def calculate_grid_size(self):
        normalized_obstacles = [self.to_grid_coordinates((x, y)) for x, y, _ in self.obstacles_np]
        all_x = [self.start[0], self.goal[0]] + [x for x, _ in normalized_obstacles]
        all_y = [self.start[1], self.goal[1]] + [y for _, y in normalized_obstacles]
        grid_width = max(all_x) + 1
        grid_height = max(all_y) + 1
        return grid_width, grid_height

    def is_within_obstacle(self, grid_point):
        # Convert the grid point back to real-world coordinates
        real_point = self.to_real_coordinates(grid_point)
        is_hit_wall = not self.path_instance.is_inside(np.array(real_point))
        is_hit_obs = self.path_instance.is_atr_in_obstacles(np.array(real_point))
        # for obstacle in self.obstacles_np:
        #     center_x, center_y, radius = obstacle
        #     if (real_point[0] - center_x) ** 2 + (real_point[1] - center_y) ** 2 <= radius ** 2:
        #         return True

        return is_hit_obs or is_hit_wall
    
    def heuristic(self, node, goal):
        # Convert the node from grid coordinates to real-world coordinates
        real_node = self.to_real_coordinates(node)

        # Standard heuristic (e.g., Euclidean distance to the goal)
        standard_heuristic = np.linalg.norm(np.array(real_node) - np.array(self.to_real_coordinates(goal)))

        reference_path_weight = 1.0  # Adjust this value as needed 0.01 is good
        if self.reference_path is not None and len(self.reference_path) > 0:
            closest_distance = min(np.linalg.norm(np.array(real_node) - np.array(point)) for point in self.reference_path)
            path_heuristic = np.exp(-closest_distance) * reference_path_weight
        else:
            path_heuristic = 0
        # Combine heuristics
        return standard_heuristic + path_heuristic
    
    def initialize(self):
        self.x_offset, self.y_offset = self.calculate_offsets()
        self.start = self.to_grid_coordinates(self.current_position)
        self.goal = self.to_grid_coordinates(self.path_instance.waypoints[-1])
        self.obstacles_np = self.path_instance.obstacles_np
        self.grid_size = self.calculate_grid_size()
        self.grid_costs = self.calculate_grid_costs()
        self.open_list = set()
        self.closed_list = set()
        self.parents = {}
        self.g_values = {}
        self.path = []
        for x in range(self.grid_size[0]):
            for y in range(self.grid_size[1]):
                self.g_values[(x, y)] = float('inf')
                self.parents[(x, y)] = None
        self.g_values[self.goal] = 0
        self.open_list.add((self.heuristic(self.goal, self.start), self.goal))        
    
    def get_neighbors(self, node):
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # 4-way connectivity
        neighbors = []
        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])

            # Check if neighbor is within the grid boundaries
            if 0 <= neighbor[0] < self.grid_size[0] and 0 <= neighbor[1] < self.grid_size[1]:
                if not self.is_within_obstacle(neighbor):
                    neighbors.append(neighbor)
        return neighbors


    def process_node(self):
        while self.open_list:
            _, current = min(self.open_list)
            if current == self.start:
                break
            self.open_list.remove((_, current))
            self.closed_list.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_list:
                    continue
                tentative_g_value = self.g_values[current] + self.move_cost(current, neighbor) + self.grid_costs[neighbor]
                if tentative_g_value < self.g_values[neighbor]:
                    self.g_values[neighbor] = tentative_g_value
                    self.parents[neighbor] = current
                    self.open_list.add((self.g_values[neighbor] + self.heuristic(neighbor, self.start), neighbor))

    def move_cost(self, current, neighbor):
        return 0  # Assuming uniform cost

    def reconstruct_path(self):
        node = self.start
        if node in self.parents:
            while self.parents[node] is not None:
                self.path.append(self.to_real_coordinates(node))
                node = self.parents[node]
            self.path.append(self.to_real_coordinates(self.goal))
        # self.path.reverse()
    
    def smooth_path(self, points_needed=300, smoothing=0.05):
        if len(self.path) < 2:
            return
        
        path_np = np.array(self.path)
        # Calculate the cumulative distance (arc length) for each point
        distance = np.cumsum(np.sqrt(np.sum(np.diff(path_np, axis=0)**2, axis=1)))
        distance = np.insert(distance, 0, 0)  # Start with distance 0 at the first point
        if len(np.unique(distance)) <= 3:
            return 
        # Extract x and y coordinates
        x = path_np[:, 0]
        y = path_np[:, 1]

        # # Generate new points along the spline curves
        distance_new = np.linspace(0, distance[-1], points_needed)
        spline_x = UnivariateSpline(distance, x, s=smoothing)  # Adjust s for more or less smoothing
        spline_y = UnivariateSpline(distance, y, s=smoothing)  # Adjust s for more or less smoothing

        x_new = spline_x(distance_new)
        y_new = spline_y(distance_new)
        self.path_smoothed = np.array([x_new, y_new]).T
    
    def find_path(self, n_points):
        self.initialize()
        self.process_node()
        self.reconstruct_path()
        self.smooth_path(n_points)
        return self.path_smoothed
    
    def re_run_pathfinding(self, n_points):
        # Re-run the pathfinding from the current position
        self.initialize()
        self.start = self.to_grid_coordinates(self.current_position)
        self.process_node()
        self.reconstruct_path()
        self.smooth_path(n_points)
        return self.path
    
    def plot_path(self, ax):
        # Create a figure and a grid to plot
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_aspect('equal')
        ax.set_title('Astar planner')
        self.path_instance.render(ax)
        ax.scatter(self.current_position[0], self.current_position[1], color='green', alpha=0.5)
        # ax.scatter([node[0] for node in self.path], [node[1] for node in self.path], color='blue', alpha=0.5)
        ax.scatter([node[0] for node in self.path_smoothed], [node[1] for node in self.path_smoothed], color='orange', alpha=0.5)

if __name__ == "__main__":
    args = parse_args()
    # Example usage
    params = env_param()
    params.path_param.consider_width = True
    params.path_param.No = 3

    test_path = Path(params.path_param, params.atr_param.atr_linear_vel_max)
    # save the path
    # test_path.reset()
    # pickle.dump(test_path, open("path_instance.pkl", "wb"))
    test_path = pickle.load(open("path_instance.pkl", "rb"))
    current_position = test_path.even_trajectory[0]
    # pickle.dump(path_instance, open("path_instance.pkl", "wb"))
    # is_hit_wall = path_instance.is_inside(np.array([3.118, -3.364]))
    # is_hit_wall = path_instance.walls_LinearRing.contains(np.array([3.118, -3.364]))
    # print(is_hit_wall)
    dstar = AStar(test_path, current_position)
    path = dstar.find_path(len(test_path.even_trajectory) - 1)
    # plt.figure()



    fig, ax = plt.subplots()
    if args.dynamic:
        for id in range(len(test_path.even_trajectory)):
            ax.clear()
            dstar.update_environment(test_path.even_trajectory[id])
            # number of points left in the even_trajectory
            n_points = len(test_path.even_trajectory) - id
            path = dstar.re_run_pathfinding(n_points)
            last_path = path
            dstar.plot_path(ax)
            plt.pause(0.01)
    else:
        dstar.plot_path(ax)
        plt.show()
        

    # plt.show()