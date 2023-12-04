import time
import numpy as np
import scipy.interpolate as interp
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.interpolate import CubicSpline
from scipy.integrate import cumtrapz
from scipy.spatial import cKDTree
from shapely.geometry import Polygon, Point, LineString, LinearRing
import pickle
from factory_env.envs.parameters import path_param

np.seterr(all='ignore')
np.set_printoptions(precision=3, suppress=True)

starttime = time.time()
class Path():
    def __init__(self,
                 params: path_param,
                 ghost_max_vel=0.1,
                 atr_trackwidth=0.48):
        
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
        self.ref_point_index = 0
        self.atr_radius = atr_trackwidth / 2
        
    def generate_waypoints_not_back(self, Nw, Lp):
        """
        This method generates random waypoints that won't go back
        """
        # Generate the starting point
        theta_start = np.random.uniform(0, 2 * np.pi)
        p_start = 0.5 * Lp * np.array([np.cos(theta_start), np.sin(theta_start)])
        waypoints = [p_start]

        # Calculate the average segment length
        avg_segment_length = Lp / (Nw - 1)

        # Generate the remaining waypoints
        for i in range(1, Nw):  # Nw - 1 to leave space for the last waypoint (p_end)
            # Get the previous waypoint and the direction of the previous segment
            prev_waypoint = waypoints[-1]
            if i == 1:
                prev_dir = np.array([1, 0])
            else:
                prev_dir = prev_waypoint - waypoints[-2]
            
            # Generate a new waypoint in the forward direction
            while True:
                # Generate a random direction vector
                direction = np.random.rand(2) - 0.5
                direction /= np.linalg.norm(direction)

                # Check if the new waypoint is in the forward direction
                if np.dot(direction, prev_dir) >= 0:
                    break
            
            # Generate a new waypoint at the average segment length and add it to the list
            new_waypoint = prev_waypoint + avg_segment_length * direction
            waypoints.append(new_waypoint)

        self.waypoints =  np.array(waypoints)

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
                # r_obst = np.random.poisson(mu_r)
                if self.params.dynamic_obstacles_r:
                    r_obst = np.clip(np.random.normal(mu_r, 0.1), 0.1, 1)
                else:
                    r_obst = mu_r # temporary using a fix radius
                # Add obstacle (p_obst, r_obst) to environment
                obstacles.append((p_obst, r_obst))

        return p, obstacles
    
    def arc_length(self, path, t, dt):
        derivative = path.derivative()(t)
        return np.sqrt(np.sum(derivative**2, axis=1)) * dt
    
    def generate_path(self):
        self.generate_waypoints_not_back(self.Nw, self.Lp)
        path, self.obstacles = self.generate_path_colav_environment(self.No, self.Nw, self.Lp, self.mu_r, self.sigma_d)
        self.obstacles_np = np.array([list(t[0]) + [t[1]] for t in self.obstacles])

        # Calculate even trajectory points
        t = np.linspace(0, 1, 1000)
        dt = t[1] - t[0]
        cumulative_arclength = np.cumsum(self.arc_length(path, t, dt))
        point_distance = self.interval
        total_arclength = cumulative_arclength[-1]
        num_points = int(np.ceil(total_arclength / point_distance))
        even_t = np.zeros(num_points)
        current_arclength = 0
        for i in range(1, num_points):
            current_arclength += point_distance
            even_t[i] = np.interp(current_arclength, cumulative_arclength, t)
            
        # get even trajectory
        self.even_trajectory = path(even_t)
        self.even_trajectory = np.vstack((self.even_trajectory, self.waypoints[-1])) # add the last waypoint into even_trajectory
        self.trejectory_kd_tree = cKDTree(self.even_trajectory.copy())
        
        # Calculate the length of the trajectory at each point
        differences = np.linalg.norm(np.diff(self.even_trajectory, axis=0), axis=1)
        self.trajectory_length_at_each_point = np.cumsum(differences)
        self.trajectory_length_at_each_point = np.hstack((0, self.trajectory_length_at_each_point)) # add the first point in even_trajectory
        
        # self.even_trajectory = self.even_trajectory[1:] # remove the first point in even_trajectory after calculate the trajectory length
        
        # get ghost ATR trajectory
        ghost_velocity = total_arclength / self.target_finishing_time
        point_distance = ghost_velocity * self.interval
        total_steps = int(np.ceil(total_arclength / point_distance))
        even_atr_t = np.zeros(total_steps)
        current_arclength = 0
        for i in range(1, total_steps):
            current_arclength += point_distance
            even_atr_t[i] = np.interp(current_arclength, cumulative_arclength, t)
        self.ghost_trajectory = path(even_atr_t)
        self.ghost_trajectory = np.vstack((self.ghost_trajectory, self.waypoints[-1])) # add the last waypoint into even_trajectory
        # self.ghost_trajectory = self.ghost_trajectory[1:]
        
        # Calculate yaw angles at each point
        path_derivative = path.derivative()(even_t)
        self.yaw_angles = np.arctan2(path_derivative[:, 1], path_derivative[:, 0])
        self.yaw_angles = np.hstack((self.yaw_angles, self.yaw_angles[-1])) # add the last yaw angle into yaw_angles
        
        path_derivative = path.derivative()(even_atr_t)
        self.ghost_yaw_angles = np.arctan2(path_derivative[:, 1], path_derivative[:, 0])
        self.ghost_yaw_angles = np.hstack((self.ghost_yaw_angles, self.ghost_yaw_angles[-1])) # add the last yaw angle into yaw_angles
    
    def generate_walls(self):
        """
        Generate the boundary of the environment
        """
        points = self.even_trajectory.copy()
        # Shift the trajectories
        shifted_points_up, shifted_points_down = self.shift_trajectory_local(points, self.shift_distance)
        # Smooth and resample the shifted trajectories
        target_distance = self.interval
        resampled_shifted_points_up, resampled_shifted_points_down = self.smooth_and_resample_trajectories(shifted_points_up, shifted_points_down, target_distance)

        # Extend the resampled trajectories
        extended_resampled_shifted_points_up = self.extend_trajectory(resampled_shifted_points_up, self.extend_length)
        extended_resampled_shifted_points_down = self.extend_trajectory(resampled_shifted_points_down, self.extend_length)

        # Create lines connecting the starting points and ending points
        start_line = self.create_line_between_points(extended_resampled_shifted_points_up[0], extended_resampled_shifted_points_down[0], point_distance=self.interval)
        end_line = self.create_line_between_points(extended_resampled_shifted_points_up[-1], extended_resampled_shifted_points_down[-1], point_distance=self.interval)
        
        if not self.params.without_walls:
            self.wall_up = extended_resampled_shifted_points_up
            self.wall_down = extended_resampled_shifted_points_down
            self.start_line = start_line
            self.end_line = end_line

            # This list is for the normal distance calculation
            # Yeah I know it looks ugly and provides identical information with the walls_kdTree
            # May tune it in the furture
            self.walls = [self.wall_up, self.wall_down, self.start_line, self.end_line]
            exterior =  np.vstack([self.start_line, self.wall_up, self.end_line[::-1], self.wall_down[::-1]])
            # self.generate_kdTree_attributes()
            polygon = Polygon(exterior)
            cleaned_polygon = polygon.buffer(0)
            self.bounding_box_polygon = cleaned_polygon
            x, y = self.bounding_box_polygon.exterior.xy
            self.walls_stack = np.column_stack((x, y))
            self.generate_kdTree_attributes()
            self.walls_LinearRing = LinearRing(self.walls_stack)
        else:
            self.walls_LinearRing = None
            
    def generate_kdTree_attributes(self):
        """
        Generate cKDTree useful variables for querying the closest point in the walls
        """
        self.walls_kd_tree = cKDTree(self.walls_stack)
        # self.walls_stack = np.vstack([self.wall_up, self.wall_down, self.start_line, self.end_line])
        # self.starting_indices = [0, len(self.wall_up), 
        #             len(self.wall_up) + len(self.wall_down), 
        #             len(self.wall_up) + len(self.wall_down) + len(self.start_line)]
        # self.walls_kd_tree = cKDTree(self.walls_stack)

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

    def local_orthogonal_directions(self, points):
        """
        Calculate the local orthogonal directions
        """
        diff = np.diff(points, axis=0)
        normalized_diff = diff / np.linalg.norm(diff, axis=1, keepdims=True)
        orthogonal_directions = np.array([-normalized_diff[:, 1], normalized_diff[:, 0]]).T
        return orthogonal_directions

    def shift_trajectory_local(self, points, distance, num_points=None):
        """
        Shift the reference trajectory in the local orthogonal directions up and down.
        It will return two new trajectories
        """
        orthogonal_directions = self.local_orthogonal_directions(points)
        # Repeat the last orthogonal direction for the last point
        orthogonal_directions = np.vstack([orthogonal_directions, orthogonal_directions[-1, :]])
        shifted_points_up = points + distance * orthogonal_directions
        shifted_points_down = points - distance * orthogonal_directions

        if num_points is None:
            num_points = len(points)

        # Interpolate the shifted points to maintain even separation
        t_original = np.linspace(0, 1, len(points))
        t_new = np.linspace(0, 1, num_points)

        interpolator_up = interp1d(t_original, shifted_points_up, axis=0, kind='linear')
        interpolated_shifted_points_up = interpolator_up(t_new)

        interpolator_down = interp1d(t_original, shifted_points_down, axis=0, kind='linear')
        interpolated_shifted_points_down = interpolator_down(t_new)

        return interpolated_shifted_points_up, interpolated_shifted_points_down

    def smooth_and_resample_trajectories(self, trajectory1, trajectory2, target_distance):
        """
        Smooth and resample the walls
        """
        def resample_trajectory(trajectory, target_distance):
            # Compute the distances between consecutive points
            distances = np.sqrt(np.sum(np.diff(trajectory, axis=0)**2, axis=1))
            # Compute the cumulative distance along the trajectory
            cum_dist = np.concatenate(([0], np.cumsum(distances)))
            total_distance = cum_dist[-1]

            # Calculate the number of new points for the resampled trajectory
            num_points = int(np.ceil(total_distance / target_distance))
            t_new = np.linspace(0, total_distance, num_points)

            # Interpolate the trajectory using cubic splines
            spline = CubicSpline(cum_dist, trajectory)
            resampled_trajectory = spline(t_new)

            return resampled_trajectory
        
        # Resample both trajectories with the target distance
        resampled_trajectory1 = resample_trajectory(trajectory1, target_distance)
        resampled_trajectory2 = resample_trajectory(trajectory2, target_distance)

        return resampled_trajectory1, resampled_trajectory2
    
    def extend_trajectory(self, points, extend_length):
        """
        Extend the starting and ending points of the trajectory.
        """
        distance = np.linalg.norm(points[3] - points[2])
        start_direction = (points[1] - points[0]) / np.linalg.norm(points[1] - points[0])
        end_direction = (points[-1] - points[-2]) / np.linalg.norm(points[-1] - points[-2])
        num_points_to_add = int(extend_length / distance)
        start_points = [points[0] - start_direction * distance * i for i in range(1, num_points_to_add + 1)][::-1]
        end_points = [points[-1] + end_direction * distance * i for i in range(1, num_points_to_add + 1)]

        return np.vstack([start_points, points, end_points])

    def create_line_between_points(self, point_a, point_b, point_distance=0.1):
        distance = np.linalg.norm(point_b - point_a)
        num_points = int(np.ceil(distance / point_distance)) + 1
        t = np.linspace(0, 1, num_points)
        line_points = np.array([(1 - t_i) * point_a + t_i * point_b for t_i in t])
        return line_points

    def minimum_distance_to_walls(self, point, method: str = 'kdtree', num_of_closest_points=2):
        """
        Return the minimum distance to the nearest wall and that closest point
        
        ### Parameters
        - `point`: np.array([x, y])
        - `method`: str, 'kdtree' or 'normal'
        - `num_of_closest_points`: int, how many closest points on the walls are needed
        
        ### Returns
        - `min_distance`: float
        - `closest_points`: list
        
        ### Example:
        >>> point = np.array([0, 0])
        >>> min_distance, closest_point = self.minimum_distance_to_walls(point)
        """
        if method == 'kdtree':
            min_distance, index = self.walls_kd_tree.query(point, num_of_closest_points) # (2,), (2,)
            closest_points = self.walls_stack[index]

            # closest_trajectory_index = np.searchsorted(self.starting_indices, index, side="right") - 1    
            # # Convert to list for element-wise operation
            # closest_trajectory_index_list = closest_trajectory_index.tolist()
            # starting_indices_for_each_point = [self.starting_indices[i] for i in closest_trajectory_index_list]
            # closest_point_index = index - np.array(starting_indices_for_each_point)
            # closest_points = [self.walls[trajectory_index][point_index] for trajectory_index, point_index in zip(closest_trajectory_index_list, closest_point_index.tolist())]
            return min_distance, closest_points
            # print(f"kdtree: {min_distance, self.walls[closest_trajectory_index][closest_point_index]}")
        elif method=='normal':        
            min_distance = float('inf')
            closest_trajectory_index = -1
            closest_point_index = -1

            for i, trajectory in enumerate(self.walls):
                distances = np.linalg.norm(trajectory - point, axis=1)
                current_min_distance = np.min(distances)
                current_closest_point_index = np.argmin(distances)

                if current_min_distance < min_distance:
                    min_distance = current_min_distance
                    closest_trajectory_index = i
                    closest_point_index = current_closest_point_index
            # print(f"noraml way: {min_distance, self.walls[closest_trajectory_index][closest_point_index]}")
            return min_distance, self.walls[closest_trajectory_index][closest_point_index]
    
    def is_waypoints_in_obstacles(self):
        """
        Check if the waypoints are in the obstacles
        ### Returns
        - `is_in_obstacles`: False if not in obstacles, True if in obstacles
        """
        for i in range(self.waypoints.shape[0]):
            for j in range(self.obstacles_np.shape[0]):
                dist = np.sqrt((self.waypoints[i, 0] - self.obstacles_np[j, 0])**2 + (self.waypoints[i, 1] - self.obstacles_np[j, 1])**2)
                if dist <= self.obstacles_np[j, 2]:
                    return True
        return False
    
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

    def is_crossed(self):
        """
        Check if the trajectory is crossed with the walls. Also check if the start and end lines cross trajectory.
        But it cannot avoid the crossing of one trajectory itself.

        ### Returns
        - `crossing`: False if not crossed, True if crossed
        """
        line1 = LineString(self.even_trajectory)
        line2 = LineString(self.wall_up)
        line3 = LineString(self.wall_down)
        line4 = LineString(self.start_line)
        line5 = LineString(self.end_line)

        trajectory1 = [line1, line2, line3]
        trajectory2 = [line1, line4, line5]
        crossing = False
        
        for i, traj1_i in enumerate(trajectory1):
            for j, traj1_j in enumerate(trajectory1[i + 1:], i + 1):
                if traj1_i.intersects(traj1_j):
                    crossing = True
                    break
            if crossing:
                break

        for i, traj2_i in enumerate(trajectory2):
            for j, traj2_j in enumerate(trajectory2[i + 1:], i + 1):
                if traj2_i.intersects(traj2_j):
                    crossing = True
                    break
            if crossing:
                break

        return crossing
    
    def is_inside(self, point):
        """
        Check if a point is inside the boundary
        
        ### Parameters
        point: np.array([x, y])
        
        ### Returns
        is_inside: bool
        
        ### Example
        >>> point = np.array([0, 0])
        >>> is_inside = self.is_inside(point)
        """
        is_inside = self.bounding_box_polygon.contains(Point(point))
        return is_inside

    def is_arrived(self, point, tolerance=1):
        if np.linalg.norm(point - self.waypoints[-1]) < self.interval * tolerance:
            return True
        else:
            return False
    
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
                # print(f"JUMPED: {index}")
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
        
    def render(self, ax, if_yaw_angle=False, no_plotting_walls=False):
        # plot waypoints
        ax.scatter(self.waypoints[0,0], self.waypoints[0,1], s=100, c='k',  label="Start")
        ax.scatter(self.waypoints[-1,0], self.waypoints[-1,1], s=100, c='r', label="End")
        ax.scatter(self.waypoints[:, 0], self.waypoints[:, 1], c='b', marker='x',label="Waypoints")
        ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], 'r', alpha=0.2)
        
        # Plot the trajectory
        ax.scatter(self.even_trajectory[:, 0], self.even_trajectory[:, 1], s=2, marker='.', label="Trajectory", rasterized=True)
        
        for idx, obs in enumerate(self.obstacles):
            p_obst, r_obst = self.obstacles_np[idx, :2], self.obstacles_np[idx, 2]
            circle = plt.Circle((p_obst[0], p_obst[1]), r_obst, color='r', alpha=0.5)
            ax.add_patch(circle)
            ax.text(p_obst[0], p_obst[1], idx+1, fontsize=12)
        if not self.params.without_walls and not no_plotting_walls:
            # ax.scatter(self.wall_up[:, 0], self.wall_up[:, 1], s=2, marker='o', rasterized=True)
            # ax.scatter(self.wall_down[:, 0], self.wall_down[:, 1], s=2, marker='o', rasterized=True)
            # plt.scatter(query_point[0], query_point[1], s=100, marker='x', label='Query Point')
            # plt.scatter(trajectories[closest_trajectory_index][closest_point_index, 0], trajectories[closest_trajectory_index][closest_point_index, 1], s=100, marker='x', label='Closest Point')
            # ax.scatter(self.start_line[:, 0], self.start_line[:, 1], s=2, marker='.', rasterized=True)
            # ax.scatter(self.end_line[:, 0], self.end_line[:, 1], s=2, marker='.', rasterized=True)
            ax.scatter(self.walls_stack[:, 0], self.walls_stack[:, 1], s=2, marker='.', rasterized=True)
        if self.plot_error:
            ax.scatter(self.query_point[0], self.query_point[1], s=10, marker='x', label='Query Point')
            ax.scatter(self.closest_point_to_trajectory[0], self.closest_point_to_trajectory[1], s=10, marker='o', label='Closest Point')
            ax.plot((self.query_point[0], self.s_global[0]), (self.query_point[1], self.s_global[1]), 'b--', label="cross-track error")
            ax.plot((self.s_global[0], self.closest_point_to_trajectory[0]), (self.s_global[1], self.closest_point_to_trajectory[1]), 'b--', linewidth=1, label="along-track error")
        
        # ax.legend()
        # ax.xlabel("x")
        # ax.ylabel("y")
        # ax.title("Trajectory and Obstacles and boundary")
        # ax.axis("equal")
        # ax.grid()
        
        if if_yaw_angle:
            fig, ax = plt.subplots()
            yaw_angles_in_degrees = np.rad2deg(self.yaw_angles)
            plt.plot(self.trajectory_length_at_each_point, yaw_angles_in_degrees, 'b-', label="Yaw angle")
            plt.legend()
            plt.xlabel("length of the trajectory (m)")
            plt.ylabel("yaw angle (degree)")
            plt.title("Yaw angle along the reference trajectory")
            plt.grid()
            
        
    
    def print_shape(self):
        print("Shape of the variables")
        print(f"shape of waypoints: {self.waypoints.shape}")
        print(f"shape of even_trajectory: {self.even_trajectory.shape}")
        print(f"length of the reference trajectory: {round(self.trajectory_length_at_each_point[-1], 3)} m")
        print(f"shape of ghost_trajectory: {self.ghost_trajectory.shape}")
        print(f"shape of yaw_angles: {self.yaw_angles.shape}")
        print(f"shape of ghost_yaw_angles: {self.ghost_yaw_angles.shape}")
        print(f"shape of obstacles in ndarray format: {self.obstacles_np.shape}")
        print(f"number of obstacles: {len(self.obstacles)}")
        if not self.params.without_walls:
            print(f"shape of walls: {self.wall_up.shape}")
        # print("")
    
    def reset(self):
        self.plot_error = False
        self.ref_point_index = 0
        self.generate_path()
        self.generate_walls()
        if not self.params.without_walls:
            if self.is_crossed():
                self.reset()
        # if self.is_waypoints_in_obstacles():
        #     self.reset()
