import json
import numpy as np  
import math
import matplotlib.pyplot as plt
import scipy.interpolate as interp
from scipy.spatial import cKDTree

# from cubic_spline import Spline2D
from scipy.interpolate import CubicSpline
from factory_env.envs.parameters import path_param


class ATRPath():
    """
    ### Parameters
    - `path_dir`: str, the path of the json file, default: "./config/nodes_tuve.json"
    - `path_type`: str, the type of the path, default: "spline", options: ["spline", "straight", "pchip"]
    - `trajectory_point_interval`: float, the interval of the trajectory points, default: 0.1
    - `if_render_text`: bool, if render the text of the waypoints, default: True
    
    ### Example
    ```python
    atrPath = ATRPath()
    >>> data_synthesis(4, 282): | inteploated_x |
                                | inteploated_y |
                                | trajectory_lengths |
                                | yaw_angles |
    >>> shape of data_synthesis: (4, 282)
    >>> shape of waypoints_x: (14,)
    >>> shape of waypoints(14, 2)
    >>> shape of even_trajectory: (282, 2)
    >>> shape of yaw_angles: (282,)
    ```
    """
    metadata = {"path_type": ["spline", "straight", "pchip"]}
    def __init__(self, 
                 params: path_param,
                 path_dir: str = "./config/nodes_tuve.json",
                 path_name: str = "OnePath",
                 path_type: str = "spline",
                 atr_trackwidth=0.48,
                 ghost_max_vel=0.1,
                 if_render_text=True,
                 ):
        
        assert path_type in self.metadata["path_type"]
        
        self.path_dir = path_dir
        self.path_type = path_type
        self.if_render_text = if_render_text
        self.data_synthesis = None

        self.params = params
        self.interval=params.trajectory_point_interval
        self.No=params.No
        self.Nw=params.Nw
        self.Lp=params.Lp
        self.mu_r=params.mu_r
        self.sigma_d=params.sigma_d
        self.shift_distance=params.shift_distance
        self.extend_length=params.extend_length
        self.look_ahead_distance=params.look_ahead_distance # Lp * 0.2
        self.target_finishing_time=params.target_finishing_time
        
        self.atr_max_vel=ghost_max_vel
        self.atr_radius = atr_trackwidth / 2

        self.ref_point_index = 0

        with open(self.path_dir, "r") as file:
            # Load the JSON content into a Python dictionary
            data = json.load(file)

            self.paths = data[path_name]
            self.nodes = data["Nodes"]
            self.path_waypoints = []
        self.construct_waypoints()
        if self.path_type == 'spline':
            self.generate_spline_trajectory()
        elif self.path_type == 'straight':
            self.generate_straight_trajectory()
        elif self.path_type == 'pchip':
            self.generate_pchip_trajectory()  
        self.generate_path_colav_environment(self.No, self.Nw, self.Lp, self.mu_r, self.sigma_d)
        self.synthesis()
    
    def construct_waypoints(self):
        node_positions = {node['id']: np.array(node['position'][:2]) for node in self.nodes}

        for path in self.paths:
            seq = 0
            self.path_id = path['id']
            self.node_ids = path['graph']
            self.path_waypoints = [node_positions[node_id] for node_id in self.node_ids]
            if(self.if_render_text):
                for wp in self.path_waypoints:
                    seq += 1
                    plt.text(wp[0], wp[1], f'{seq}, {wp}', fontsize=8)
            self.path_waypoints = np.array(self.path_waypoints)
            self.waypoints_x = self.path_waypoints[:, 0]
            self.waypoints_y = self.path_waypoints[:, 1]
        self.waypoints = np.column_stack((self.waypoints_x, self.waypoints_y))
        
    def generate_spline_trajectory(self):
        # Calculate the arc length
        ds = np.sqrt(np.diff(self.waypoints_x)**2 + np.diff(self.waypoints_y)**2)
        s = np.hstack(([0], np.cumsum(ds)))

        # Create the cubic spline using the arc length as the parameter
        cs = CubicSpline(s, self.waypoints_y, bc_type='clamped')
        cs_x = CubicSpline(s, self.waypoints_x, bc_type='clamped')

        # Calculate the total trajectory length and number of points for interpolation
        total_length = s[-1]
        
        num_points = int(total_length / self.interval ) + 1
        # Generate interpolation points
        self.interp_trajectory_lengths = np.linspace(s[0], s[-1], num=num_points)
        self.interpolated_y = cs(self.interp_trajectory_lengths)
        self.interpolated_x = cs_x(self.interp_trajectory_lengths)

        # Calculate the yaw angle using the derivatives of the cubic spline
        dx = cs_x.derivative(1)(self.interp_trajectory_lengths)
        dy = cs.derivative(1)(self.interp_trajectory_lengths)
        self.yaw_angles = np.arctan2(dy, dx)
        self.yaw_angles[0] = self.yaw_angles[1]
        self.yaw_angles[-1] = self.yaw_angles[-2]
        
    
    def generate_straight_trajectory(self):
        
        interpolated_points = [self.waypoints[0]]
        for i in range(1, len(self.waypoints)):
            dist = np.linalg.norm(self.waypoints[i] - self.waypoints[i-1])
            num_points = int(np.ceil(dist / self.interval))
            t_values = np.linspace(0, 1, num=num_points)
            
            for t in t_values:
                point = self.waypoints[i-1] + t * (self.waypoints[i] - self.waypoints[i-1])
                if not np.allclose(point, interpolated_points[-1]):
                    interpolated_points.append(point)
        interpolated_points = np.array(interpolated_points)
        
        delta_points = np.diff(interpolated_points, axis=0)
        segment_distances = np.linalg.norm(delta_points, axis=1)
        cumulative_distances = np.cumsum(segment_distances)
        self.interp_trajectory_lengths = np.insert(cumulative_distances, 0, 0)  # Add a zero at the beginning for the starting point
    
        self.yaw_angles = np.arctan2(delta_points[:, 1], delta_points[:, 0])
        self.yaw_angles = np.append(self.yaw_angles, self.yaw_angles[-1])
        self.interpolated_x = np.squeeze(interpolated_points[:, 0])
        self.interpolated_y = np.squeeze(interpolated_points[:, 1])
    
    def generate_pchip_trajectory(self):
        def arc_length(path, t, dt):
            derivative = path.derivative()(t)
            return np.sqrt(np.sum(derivative**2, axis=1)) * dt
        
        # waypoints = np.column_stack((x, y))
        path = interp.PchipInterpolator(np.linspace(0, 1, len(self.waypoints)), self.waypoints, axis=0)
        # Calculate even trajectory points
        t = np.linspace(0, 1, 100)
        dt = t[1] - t[0]
        cumulative_arclength = np.cumsum(arc_length(path, t, dt))
        total_arclength = cumulative_arclength[-1]
        num_points = int(np.ceil(total_arclength / self.interval))
        even_t = np.zeros(num_points)
        current_arclength = 0
        for i in range(1, num_points):
            current_arclength += self.interval
            even_t[i] = np.interp(current_arclength, cumulative_arclength, t)
        even_trajectory = path(even_t)
        
        # calculate yaw angles at each point
        even_arclength = np.interp(even_t, t, cumulative_arclength)
        # Calculate yaw angles at each point
        path_derivative = path.derivative()(even_t)
        yaw_angles = np.arctan2(path_derivative[:, 1], path_derivative[:, 0])

        self.interp_trajectory_lengths = even_arclength
        self.yaw_angles = yaw_angles
        # self.yaw_angles = np.append(self.yaw_angles, self.yaw_angles[-1])
        self.interpolated_x = np.squeeze(even_trajectory[:, 0])
        self.interpolated_y = np.squeeze(even_trajectory[:, 1])
    
    def generate_path_colav_environment(self, No, Nw, Lp, mu_r, sigma_d):
    
        # Create smooth arc length parameterized path using PCHIP
        p = interp.PchipInterpolator(np.linspace(0, 1, Nw), self.waypoints, axis=0)

        obstacles = []

        if not self.params.without_obstacles:
            # Generate No obstacles
            for _ in range(No):
                # Draw arclength omega_obst from Uniform(0.1 * Lp, 0.9 * Lp)
                omega_obst = np.random.uniform(0.1 * Lp, 0.9 * Lp)

                # Map omega_obst to normalized arclength
                norm_omega_obst = omega_obst / Lp

                # Draw obstacle displacement distance d_obst from N(0, sigma_d^2)
                d_obst = np.random.normal(0, sigma_d**2)

                # Path angle gamma_obst
                gamma_obst = np.arctan2(p.derivative()(norm_omega_obst)[1], p.derivative()(norm_omega_obst)[0])

                # Obstacle position
                p_obst = p(norm_omega_obst) + d_obst * np.array([np.cos(gamma_obst - np.pi/2), np.sin(gamma_obst - np.pi/2)])

                # Draw obstacle radius r_obst from Poisson(mu_r)
                # r_obst = np.clip(np.random.normal(mu_r, 0.3), 0.1, 1)
                r_obst = mu_r # temporary using a fix radius
                # Add obstacle (p_obst, r_obst) to environment
                obstacles.append((p_obst, r_obst))
        self.obstacles_np = np.array([list(t[0]) + [t[1]] for t in obstacles])

    def calculate_error_vector(self, atr_state: np.ndarray, ghost_point: np.ndarray=None):
        """
        This method will first check if the query point is within the area. 
        Then calculate the along-track error and cross-track error to the closest point 
        on the reference trajectory. This will also set the self.plot_error to True. Then the render() method
        will ploot the query_point and the errors.
        This method will also return the look-ahead point and the yaw angle of the look-ahead point.
        
        ## TODO
        - [ ] Add the yaw error
        
        ### Parameters
        - `atr_state`: np.array([x, y, yaw_angle])
        
        ### Returns
        - `eta`: np.array([s, e])
        - `index`: int, index of the closest point on the trajectory
        - `target_point`: np.array([x, y]), the look ahead point on the trajectory
        - `target_point_yaw`: float, the yaw angle of the target point on the trajectory
        
        ### Example
        >>> point = np.array([0, 0])
        >>> eta, index, target_point, target_point_yaw, look_ahead_course_error, course_error = self.calculate_error_vector(point)
        """
        point = atr_state[:2]
        heading = atr_state[2]
        # assert self.is_inside(point), "The query point is outside the boundary"
        
        if ghost_point is not None:
            pass
        
        min_distance, index = self.trejectory_kd_tree.query(point)
        distance_to_goal = self.trajectory_length_at_each_point[-1] - self.trajectory_length_at_each_point[index]
        
        # index = index + self.params.how_many_points_forward # shifted 3 points forward
        # if index >= len(self.even_trajectory):
        #     index = len(self.even_trajectory) - 1
        # self.closest_point_to_trajectory = self.even_trajectory[index]
        # # check if this point is in the obstacles
        # # If true then shift the index forward until it is not in the obstacles
        # while self.is_atr_in_obstacles(self.closest_point_to_trajectory):
        #     index += 2
        #     if index >= len(self.even_trajectory):
        #         index = len(self.even_trajectory) - 1
        #     if index < self.ref_point_index:
        #         index = self.ref_point_index
        #     else:
        #         self.ref_point_index = index
        #     self.closest_point_to_trajectory = self.even_trajectory[index]
        # else:
        #     if index < self.ref_point_index:
        #         index = self.ref_point_index
        #     else:
        #         self.ref_point_index = index
        # closest_point_yaw_angle = self.yaw_angles[index]

        index += self.params.how_many_points_forward
        if index >= len(self.even_trajectory):
            index = len(self.even_trajectory) - 1

        # Loop to handle when the index is in obstacles
        while self.is_atr_in_obstacles(self.even_trajectory[index]):
            index += 2
            if index >= len(self.even_trajectory):
                index = len(self.even_trajectory) - 1
            if index < self.ref_point_index:
                index = self.ref_point_index
            self.ref_point_index = index

        # After exiting the loop, prevent large jumps and going backward if the index is not in an obstacle
        if not self.is_atr_in_obstacles(self.even_trajectory[index]):
            if index > self.ref_point_index + self.params.allowed_jump_index:
                index = self.ref_point_index + self.params.allowed_jump_index
            elif index < self.ref_point_index:
                index = self.ref_point_index
            else:
                self.ref_point_index = index

        self.closest_point_to_trajectory = self.even_trajectory[index]
        closest_point_yaw_angle = self.yaw_angles[index]



        R = np.array([[np.cos(closest_point_yaw_angle), -np.sin(closest_point_yaw_angle)],
                      [np.sin(closest_point_yaw_angle), np.cos(closest_point_yaw_angle)]])
        eta = R.T @ (point - self.closest_point_to_trajectory) # eta = [s, x]->[along-track error, cross-track error]
        # print(f"eta: [s, x] = {eta}") 
        s = np.array([eta[0], 0.0])
        e = np.array([0.0, eta[1]])
        self.s_global = R @ s + self.closest_point_to_trajectory
        
        self.query_point = point
        self.plot_error = True
        # e_global = R @ e + closest_point # not needed
        index_of_look_ahead_point = int(index + self.look_ahead_distance / self.interval)
        if index_of_look_ahead_point >= len(self.even_trajectory):
            index_of_look_ahead_point = len(self.even_trajectory) - 1
        target_point = self.even_trajectory[index_of_look_ahead_point] 
        target_point_yaw = self.yaw_angles[index_of_look_ahead_point]
        # course_error is the course change needed for navigating straight towards to the look-ahead point
        course_error = np.arctan2(target_point[1] - point[1], target_point[0] - point[0]) - heading
        # make sure it is in the range of [-pi, pi]
        course_error = np.remainder(course_error + np.pi, 2 * np.pi) - np.pi
        # look_ahead_course_error is the course difference between look-ahead point yaw and the current heading
        look_ahead_course_error = target_point_yaw - heading
        look_ahead_course_error = np.remainder(look_ahead_course_error + np.pi, 2 * np.pi) - np.pi
        
        return eta, index, target_point, target_point_yaw, look_ahead_course_error, course_error, distance_to_goal
    
    def is_arrived(self, point):
        if np.linalg.norm(point - self.waypoints[-1]) < self.interval:
            return True
        else:
            return False
        
    def update_obstacles(self):
        """
        Updates the positions of obstacles by adding a random value within the speed limit to
        the x and y coordinates. Each obstacle receives a unique random change.
        The random values are scaled to not exceed the speed limit in either direction.
        The third column (other attributes) remains unchanged.
        
        :param obstacles_np: numpy array of obstacles, where each row is (x, y, ...)
        :param speed_limit: maximum change for x and y coordinates
        :return: None; the operation is done in-place on the numpy array
        """
        # Generate a random change for each obstacle's x and y coordinate
        # Values are between -0.5 and 2, then scaled down to fit within the speed limit
        # random_changes = np.random.uniform(-0.1, 0.2, size=(self.obstacles_np.shape[0], 2))
        random_changes = np.random.normal(self.params.obs_mean_vel, self.params.obs_std_vel, size=(self.obstacles_np.shape[0], 2))
        
        # Determine which obstacles will have their direction flipped
        flip_directions = np.random.rand(self.obstacles_np.shape[0]) < self.params.flip_chance
        
        # Apply the direction flip by multiplying by -1
        random_changes[flip_directions, :] *= -1
        np.clip(random_changes, -self.atr_max_vel, self.atr_max_vel, out=random_changes)
        
        scaling_factors = 0.1
        random_changes *= scaling_factors

        # Apply the random changes to each obstacle's x and y coordinates
        self.obstacles_np[:, :2] += random_changes

    def synthesis(self):
        """
        Stack the data together, into a shape of (4, num_points)
        | inteploated_x | inteploated_y | trajectory_lengths | yaw_angles |
        """
        self.data_synthesis = np.vstack((self.interpolated_x, self.interpolated_y, self.interp_trajectory_lengths ,self.yaw_angles))
        self.even_trajectory = np.column_stack((self.interpolated_x, self.interpolated_y))
        self.trejectory_kd_tree = cKDTree(self.even_trajectory.copy())
        differences = np.linalg.norm(np.diff(self.even_trajectory, axis=0), axis=1)
        self.trajectory_length_at_each_point = np.cumsum(differences)
        self.trajectory_length_at_each_point = np.hstack((0, self.trajectory_length_at_each_point)) # add the first point in even_trajectory

    def is_atr_in_obstacles(self, atr_pos):
        """
        Check if atr hits the obstacles.
        Retrun True if hit
        """
        x, y = atr_pos
        for j in range(self.obstacles_np.shape[0]):
                dist = np.sqrt((x - self.obstacles_np[j, 0])**2 + (y - self.obstacles_np[j, 1])**2)
                if self.params.consider_width:
                    if dist <= self.obstacles_np[j, 2] + self.atr_radius:
                        return True
                else:
                    if dist <= self.obstacles_np[j, 2]:
                        return True
        return False
        
    def render(self, ax):
        # ploat waypoints
        ax.plot(self.waypoints_x, self.waypoints_y, marker='o',
                     markersize=5, linestyle='-', linewidth=0.2, label=f"Path {self.path_id}")
        # plot the interpolated points
        ax.scatter(self.even_trajectory[:, 0], self.even_trajectory[:,1], s=0.5, marker='.', label='Interpolated Points')
        
        for idx, obs in enumerate(self.obstacles_np):
            p_obst, r_obst = self.obstacles_np[idx, :2], self.obstacles_np[idx, 2]
            circle = plt.Circle((p_obst[0], p_obst[1]), r_obst, color='r', alpha=0.5)
            ax.add_patch(circle)
            ax.text(p_obst[0], p_obst[1], idx+1, fontsize=12)
        # fig, ax = plt.subplots(1)
        # plt.plot(self.data_synthesis[2,:], self.data_synthesis[3,: ], 'b-', label="Yaw angle")
    
    def print_shape(self):
        print(f"shape of data_synthesis : {self.data_synthesis.shape}")
        print(f"shape of waypoints_x: {self.waypoints_x.shape}")
        print(f"shape of waypoints (numpy array, (10,2)): {self.waypoints.shape}")
        print(f"total length of the trajectory: {round(self.data_synthesis[2,-1], 3)}")
        print(f"shape of even_trajectory: {self.even_trajectory.shape}")
        print(f"shape of yaw_angles: {self.yaw_angles.shape}")
        
if __name__ == "__main__":
    # test_spline2d()
    path_dir = "/home/zhicun/code/atr_rl/Code/python_tools/FactoryEnv/factory_env/map/config/nodes_tuve.json"
    path_name = "testPath"
    atrPath = ATRPath(path_dir=path_dir, path_name=path_name, if_render_text=True, path_type='pchip')
    atrPath.render()
    atrPath.print_shape()
    plt.show()
    
    
    