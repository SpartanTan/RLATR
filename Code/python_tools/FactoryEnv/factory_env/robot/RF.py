import numpy as np
from shapely.geometry import LinearRing, LineString, Point
from factory_env.envs.parameters import env_param, sensor_param
from numba import jit, prange, njit


@njit(parallel=True)
def calc_intersections(robot_state, obstacles, beams, Sr):
    x1, y1, yaw_of_robot = robot_state[0], robot_state[1], robot_state[2]
    beam_angles = beams + yaw_of_robot
    measurements = np.full((len(beam_angles), 2), 1.0)  # Initialize with max distance
    obstacles_obs = []
    intersections = np.zeros((len(beam_angles), 2))
    
    obstacle_x = obstacles[:, 0]
    obstacle_y = obstacles[:, 1]
    obstacle_r = obstacles[:, 2]

    for i, theta in enumerate(beam_angles):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
            
        dx = x1 - obstacle_x
        dy = y1 - obstacle_y

        B = 2 * (cos_theta * dx + sin_theta * dy)
        C = dx**2 + dy**2 - obstacle_r**2
        discriminant = B**2 - 4*C

        valid_intersections = discriminant >= 0
        if valid_intersections.any():
            t1 = (-B - np.sqrt(discriminant)) / 2
            t2 = (-B + np.sqrt(discriminant)) / 2

            t_values = np.where(t1 < t2, t1, t2)  # Pick the smaller t value for each intersection
            valid_t_values = (t_values >= 0) & (t_values <= Sr)
            
            for j in np.where(valid_intersections & valid_t_values)[0]:
                t = t_values[j]
                if t < measurements[i][1]:
                    x = x1 + t * cos_theta
                    y = y1 + t * sin_theta
                    measurements[i][1] = t
                    intersections[i] = (x, y)
                    obstacles_obs.append([t, theta - yaw_of_robot, theta])

        # Fill theta value in measurements
        measurements[i][0] = theta
    return intersections, measurements, obstacles_obs

class RangeFinder():
    def __init__(self, sensor_param: sensor_param, robot_width):
        self.sensor_param = sensor_param
        self.walls = None
        self.W = robot_width

    def reset(self, walls):
        self.walls = walls

    def all_intersections(self, robot_state, obstacles):
        """
        This function calculates all intersections between the sensor beams and the obstacles and the walls.

        ### Parameters:
        - robot_pos: (x, y) tuple of robot position
        - obstacles: env.path.obstacles_np
        - sensor_angles: env.sensor.sensor_angles
        - walls: env.path.walls

        ### Returns
        - `intersections` list, [(x, y), ...]
        - `measurements` list, [(angle, distance), ...], size: (number_of_sensors, 2). Notice the angle here is the global angle of that beam
        - `obstacles_obs` list, [[distance, angle, global_angle], ...], size: (number_of_sensors, 3)
        """
        x1, y1, yaw_of_robot = robot_state[0], robot_state[1], robot_state[2]  # Assuming sensor is at the origin of the car frame
        beam_angles_global = self.sensor_param.beams + yaw_of_robot
        # Pre-compute A for all angles
        A_values = np.cos(beam_angles_global)**2 + np.sin(beam_angles_global)**2

        intersections = []
        measurements = []
        obstacles_obs = []
        
        for i, theta in enumerate(beam_angles_global):
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
            
            # Calculate intersection with walls
            x_end = x1 + self.sensor_param.Sr * np.cos(theta)
            y_end = y1 + self.sensor_param.Sr * np.sin(theta)
            beam = LineString([(x1, y1), (x_end, y_end)])
            intersection = self.walls.intersection(beam)
            
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
                    if x >= x1 and abs(t) <= self.sensor_param.Sr:
                        intersections.append((x, y))
                        measurements.append((theta, t))
                        obstacles_obs.append([t, theta - yaw_of_robot, theta])
                        intersection_found = True
                        break
                else:  # Forward direction is negative x
                    if x <= x1 and abs(t) <= self.sensor_param.Sr:
                        intersections.append((x, y))
                        measurements.append((theta, t))
                        obstacles_obs.append([t, theta - yaw_of_robot, theta])
                        intersection_found = True
                        break

            if not intersection_found:
                measurements.append((theta, self.sensor_param.Sr))
                obstacles_obs.append([self.sensor_param.Sr, theta - yaw_of_robot, theta])
                
        return intersections, measurements, obstacles_obs
    
    def all_intersections_optim(self, robot_state, obstacles):
        x1, y1, yaw_of_robot = robot_state[0], robot_state[1], robot_state[2]
        beam_angles = self.sensor_param.beams + yaw_of_robot
        intersections = []
        measurements = []
        obstacles_obs = []

        obstacles = np.array(obstacles)
        obstacle_x = obstacles[:, 0]
        obstacle_y = obstacles[:, 1]
        obstacle_r = obstacles[:, 2]

        for i, theta in enumerate(beam_angles):
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)

            dx = x1 - obstacle_x
            dy = y1 - obstacle_y

            B = 2 * (cos_theta * dx + sin_theta * dy)
            C = dx**2 + dy**2 - obstacle_r**2
            discriminant = B**2 - 4*C

            intersections_temp = []
            valid_intersections = discriminant >= 0
            if valid_intersections.any():
                t1 = (-B - np.sqrt(discriminant)) / 2
                t2 = (-B + np.sqrt(discriminant)) / 2

                t_values = np.where(t1 < t2, t1, t2)  # Pick the smaller t value for each intersection

                valid_t_values = (t_values >= 0) & (t_values <= self.sensor_param.Sr)

                x_intersections = x1 + t_values * cos_theta
                y_intersections = y1 + t_values * sin_theta

                intersections_temp.extend([(x, y, t) for x, y, t in zip(x_intersections[valid_intersections & valid_t_values], y_intersections[valid_intersections & valid_t_values], t_values[valid_intersections & valid_t_values])])

            x_end = x1 + self.sensor_param.Sr * cos_theta
            y_end = y1 + self.sensor_param.Sr * sin_theta
            beam = LineString([(x1, y1), (x_end, y_end)])
            intersection = self.walls.intersection(beam)

            if not intersection.is_empty:
                if intersection.geom_type == 'Point':
                    t_wall = ((intersection.x - x1)**2 + (intersection.y - y1)**2)**0.5
                    if t_wall <= self.sensor_param.Sr:
                        intersections_temp.append((intersection.x, intersection.y, t_wall))
                elif intersection.geom_type == 'MultiPoint':
                    for point in intersection.geoms:
                        t_wall = ((point.x - x1)**2 + (point.y - y1)**2)**0.5
                        if t_wall <= self.sensor_param.Sr:
                            intersections_temp.append((point.x, point.y, t_wall))

            if intersections_temp:
                intersections_temp.sort(key=lambda x: x[2])
                x, y, t = intersections_temp[0]
                intersections.append((x, y))  # Only append the intersection coordinates, not the distance 't'
                measurements.append((theta, t))
                obstacles_obs.append([t, theta - yaw_of_robot, theta])

            else:
                measurements.append((theta, 1.0))
                obstacles_obs.append([self.sensor_param.Sr, theta - yaw_of_robot, theta])

        return intersections, measurements, obstacles_obs

    def all_intersections_numba(self, robot_state, obstacles):
        beams = self.sensor_param.beams
        Sr = self.sensor_param.Sr

        intersections, measurements, obstacles_obs = calc_intersections(robot_state, obstacles, beams, Sr)
            
        x1, y1, yaw_of_robot = robot_state[0], robot_state[1], robot_state[2]
        beam_angles = self.sensor_param.beams + yaw_of_robot

        for i, theta in enumerate(beam_angles):
            x_end = x1 + self.sensor_param.Sr * np.cos(theta)
            y_end = y1 + self.sensor_param.Sr * np.sin(theta)
            beam = LineString([(x1, y1), (x_end, y_end)])
            intersection = self.walls.intersection(beam)

            if not intersection.is_empty:
                if intersection.geom_type == 'Point':
                    t = ((intersection.x - x1)**2 + (intersection.y - y1)**2)**0.5
                    if t < measurements[i][1]:
                        measurements[i][1] = t
                        intersections[i] = (intersection.x, intersection.y)
                elif intersection.geom_type == 'MultiPoint':
                    for point in intersection.geoms:
                        t = ((point.x - x1)**2 + (point.y - y1)**2)**0.5
                        if t < measurements[i][1]:
                            measurements[i][1] = t
                            intersections[i] = (point.x, point.y)

        return intersections, measurements, obstacles_obs
    
    def feasibility_pooling(self, x):
        """fastest"""
        # sort x in ascending order
        sorted_x = np.sort(x)

        # for each value xi in sorted x
        for xi in sorted_x:
            di = self.sensor_param.theta * xi  # calculate arc-length
            y = di / 2  # calculate opening-width
            si = False  # initialize opening found flag

            # for each value xj in x
            for xj in x:
                if xj > xi:
                    y += di
                    if y > self.W:
                        si = True
                        break
                    else:
                        y += di / 2
                        if y > self.W:
                            si = True
                            break
                    y = 0

            # if si is False for all xj in x, return xi
            if not si:
                return xi

        # return None if no feasible solution found
        return None
    
    def get_sector_measurements(self, distances):
        """
        This method generates a list containing the measurements for each sector.
        Since the number of sectors and the number of original measurements are always mismatched, for example 20 sectors and 21 measurements, we need to negelect the first measurement and then reshape the rest measurements into a 20x1 array, then add the first measeurement back to the first sector.

        ### Parameters:
        - distances: list of distances from the sensor to the obstacles

        ### Returns:
        - mes: list of measurements for each sector. Looks like [[d1, d2, ...], [d1, d2, ...], ...]
        """
        mes = np.array(distances)
        first_distance = mes[0]
        rest_distances = mes[1:].reshape(20, -1)
        rest_distances_list = rest_distances.tolist()
        rest_distances_list[0].insert(0, first_distance)
        mes = rest_distances_list
        return mes
    
    def closeness_cal(self, robot_state, obstacles):
        """
        ### Parameters:
        - `robot_state`: (x, y, theta) tuple or ndarray of robot state
        - `obstacles`: env.path.obstacles_np

        ### Returns:
        - `intersections_`: list of intersection points, [(x, y), ...]
        - `measurements` list, [(angle, distance), ...], size: (number_of_sensors, 2). Notice the angle here is the global angle of that beam
        - `fb_results`: list of feasibility pooling results, [fb1, fb2, fb3, ...]
        - `closeness`: list of closeness values, [cl1, cl2, cl3, ...]
        """
        # intersections_, measurements_, obstacles_obs_ = self.all_intersections(robot_state, obstacles)
        intersections_, measurements_, obstacles_obs_ = self.all_intersections_optim(robot_state, obstacles)
        # intersections_, measurements_, obstacles_obs_ = self.all_intersections_numba(robot_state, obstacles)
        
        angles, distances = zip(*measurements_)
        mes = self.get_sector_measurements(distances)

        fb_results = []
        closeness = []
        for i, x in enumerate(mes):
            tmp = self.feasibility_pooling(x)
            fb_results.append(tmp)
            closeness.append(1.0 - 1.0/self.sensor_param.Sr * tmp)
        # for i in measurements_:
        #     print(i)
        return intersections_, obstacles_obs_,  measurements_, fb_results, closeness