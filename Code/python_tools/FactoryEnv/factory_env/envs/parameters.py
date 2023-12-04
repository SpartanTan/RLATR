import numpy as np

class path_param:
    def __init__(self) -> None:
        self.trajectory_point_interval=0.1 # the distance between each two points on the path
        self.No=6        # number of obstacles
        self.Nw=8        # number of waypoints
        self.Lp=6        # desired length of the path
        self.Lp_std = 6 # standard length of the path
        self.mu_r=0.25   # mean of the raidus of the obstacle
        self.sigma_d=0.5 # standard deviation of the distance between the obstacle and the path
        self.shift_distance=1 # distance between the path and the walls
        self.extend_length=2.0 # how much should the wall extended at the two ends of the path
        self.look_ahead_distance=0.5 # how far should the robot look ahead
        self.target_finishing_time=30 # how long should the robot finish the path [s], used for setting ghost trajectory
        self.without_obstacles = False # Generate a path without obstacles
        self.without_walls = False # Generate a path without walls
        self.without_anything = False # Generate a path without anything
        self.allowed_jump_index = 5 # how many points can the robot jump on the path
        self.how_many_points_forward = 2 # how many points should be considered for the path following reward
        self.max_ep_steps = 500

        self.obs_mean_vel = 0.1 # mean velocity of the obstacles
        self.obs_std_vel = 1.0 # standard deviation of the velocity of the obstacles
        self.flip_chance = 0.3 # chance of flipping the velocity of the obstacles
        self.dynamic_obstacles_r = False # whether the obstacles have different radius or not
        self.dynamic_obstacles = False # whether the obstacles are dynamic or not
        self.static_map = 0 # 0: run, 1: save, 2: load
        self.consider_width = 0 # 0: not consider atr width, 1: consider atr width

    def update(self):
        if self.No == 0:
           self.without_obstacles = True
        if self.without_obstacles and self.without_walls:
            self.without_anything = True

class ATR_param:
    def __init__(self):
        self.dt = 0.1 # time step
        self.wheel_radius=0.125
        self.track_width=0.48
        self.atr_linear_vel_max = 0.2 # m/s
        self.atr_w_max = self.atr_linear_vel_max / self.wheel_radius # rad/s
        self.atr_w_min = -self.atr_w_max # rad/s
    
    def update(self):
        self.atr_w_max = self.atr_linear_vel_max / self.wheel_radius # rad/s
        self.atr_w_min = -self.atr_w_max # rad/s

class sensor_param:
    def __init__(self):
        self.distance_threshold = 0.8 # how far the robot can see
        self.sensor_angle = 240 # total angle of the sensor
        self.nsectors = 20 # how many sectors to be divided default [20]
        self.num_points_on_walls = 5 # how many points on the walls should be considered
        self.num_points_on_obstacles = 5 # how many points on the obstacles should be considered
        
        self.angle_threshold = np.deg2rad(self.sensor_angle/2) # half of the sensor angle, used for check abs(obs_angle) < angle_threshold
        self.sector_bound_angle = np.deg2rad(self.sensor_angle/2)
        self.narrow_angle = np.deg2rad(60) # angles on the circle, how much from the angle difference to the starting bound
        self.angle_inbetween = np.deg2rad(120) # angles on the circle, how much between the starting bound and the ending bound
        
        self.num_of_sectors = self.nsectors + 1
        self.sectors = np.linspace(-self.sector_bound_angle, self.sector_bound_angle, self.num_of_sectors) # angles from -angle_threshold to angle_threshold, divided into num_of_sectors sectors
        self.angles_of_sectors = self.sectors[:-1] + np.diff(self.sectors) / 2  # angles of the middle of each sector, direction for plotting the average obstacle distance in that sector

        # For rangefinder
        self.resolution = np.deg2rad(6)
        self.min_bound = -np.deg2rad(self.sensor_angle/2)
        self.max_bound = np.deg2rad(self.sensor_angle/2)
        Ss = np.deg2rad(self.sensor_angle) # total sensor span, [0, 2pi]
        self.N_of_sensors = int(np.rint(abs((self.max_bound - self.min_bound) / self.resolution))) + 1 # total number of sensors
        self.beams = np.linspace(self.min_bound, self.max_bound, self.N_of_sensors) # sensor beams
        self.beams = np.round(self.beams, 3)
        self.theta = Ss / (self.N_of_sensors - 1) # angle between neighboring sensors; should be equal to resolution
        self.Sr = 1.0 # maximum rangefinder distance

    def update(self):
        self.angle_threshold = np.deg2rad(self.sensor_angle/2)
        self.sector_bound_angle = np.deg2rad(self.sensor_angle/2)
        self.num_of_sectors = self.nsectors + 1
        self.sectors = np.linspace(-self.sector_bound_angle, self.sector_bound_angle, self.num_of_sectors) # angles from -angle_threshold to angle_threshold, divided into num_of_sectors sectors
        self.angles_of_sectors = self.sectors[:-1] + np.diff(self.sectors) / 2  # angles of the middle of each sector, direction for plotting the average obstacle distance in that sector

        self.min_bound = -np.deg2rad(self.sensor_angle/2)
        self.max_bound = np.deg2rad(self.sensor_angle/2)
        Ss = np.deg2rad(self.sensor_angle) # total sensor span, [0, 2pi]
        self.N_of_sensors = int(np.rint(abs((self.max_bound - self.min_bound) / self.resolution))) + 1 # total number of sensors
        self.beams = np.linspace(self.min_bound, self.max_bound, self.N_of_sensors) # sensor beams
        self.beams = np.round(self.beams, 3)
        self.theta = Ss / (self.N_of_sensors - 1)

class reward_param:
    def __init__(self) -> None:
        self.alpha_lambda = 1.0 # trade-off paramter
        self.beta_lambda  = 2.0# trade-off paramter
        self.gamma_theta = 4.0 # the larger, the more sensitive to the angle difference
        self.gamma_x = 0.1 # obstacle avoidance reward parameter
        self.epsilon_x = 0.1 # deadzone distance
        self.gamma_e = 0.5 # path following reward parameter 3.0 makes sensitive to the cross-track error
        self.alpha_r = 0.1 # existance reward parameter
        self.r_collision = -20000
        self.r_arrived = 20000
        self.r_terminated = -20000
        self.max_total_reward = 10000
        self.trajectory_progress_reward = 5.0
        self.lagging_frac = 3
        self.r_oa_frac = 5
        self.angle_resolution = 0.1
        self.distance_resolution = 0.05
        self.oa_result_resolution = 0.2
        self.pf_result_resolution = 0.1
        
        self.vel_window = 10
        self.duration = 0.5
        self.low_speed_threshold = 0.1
        self.low_speed_penalty_rate = 0.1
        self.max_low_speed_penalty = 5.0
        self.backward_penalty_rate = 30.0

class env_param:
    def __init__(self):
        self.path_param = path_param()
        self.reward_param = reward_param()
        self.atr_param = ATR_param()
        self.sensor_param = sensor_param()
                
        self.full_plot = True # True if want plotting the dashboad; False for just plotting the trajectory
        self.rangefinder = True # True if want to use rangefinder; False for using the sector-based sensor
        self.critical_distance = 1.2 * self.atr_param.track_width / 2
        if self.path_param.without_anything:
            self.observation_space_n = 7 + self.sensor_param.nsectors
        else:
            self.observation_space_n = 7 + self.sensor_param.nsectors
    
    def update(self):
        self.atr_param.update()
        self.sensor_param.update()
        self.path_param.update()
        if self.path_param.without_anything:
            self.observation_space_n = 7 + self.sensor_param.nsectors
        else:
            self.observation_space_n = 7 + self.sensor_param.nsectors
        self.critical_distance = 1.2 * self.atr_param.track_width / 2
        
    def print_param(self):
        print(f"Environment setup: \n\
                Path length: {self.path_param.Lp} m\n\
                Number of obstacles: {self.path_param.No}\n\
                Without obstacles: {self.path_param.without_obstacles}\n\
                Without walls: {self.path_param.without_walls}\n\
                ATR max velocity: {self.atr_param.atr_linear_vel_max} m/s\n\
                ATR max wheel speed: {self.atr_param.atr_w_max} rad/s\n\
                target finishing time: {self.path_param.target_finishing_time} s\n\
                step time: {self.atr_param.dt} s\n\
                ")
        
    def load_arguments(self, args):
        # Update path_param attributes
        self.path_param.No = args.No
        self.path_param.Nw = args.Nw
        self.path_param.Lp = args.Lp
        self.path_param.mu_r = args.mu_r
        self.path_param.sigma_d = args.sigma_d
        self.path_param.shift_distance = args.shift_distance
        self.path_param.extend_length = args.extend_length
        self.path_param.look_ahead_distance = args.look_ahead_distance
        self.path_param.target_finishing_time = args.target_finishing_time
        self.path_param.without_walls = args.without_walls
        self.path_param.how_many_points_forward = args.how_many_points_forward
        self.path_param.max_ep_steps = args.max_ep_steps
        self.path_param.obs_mean_vel = args.obs_mean_vel
        self.path_param.obs_std_vel = args.obs_std_vel
        self.path_param.flip_chance = args.flip_chance
        self.path_param.dynamic_obstacles_r = args.dynamic_obstacles_r
        self.path_param.dynamic_obstacles = args.dynamic_obstacles
        self.path_param.static_map = args.static_map
        self.path_param.consider_width = args.consider_width
        
        # Update ATR_param attributes
        self.atr_param.dt = args.dt
        self.atr_param.wheel_radius = args.wheel_radius
        self.atr_param.track_width = args.track_width
        self.atr_param.atr_linear_vel_max = args.atr_linear_vel_max

        # Update sensor_param attributes
        self.sensor_param.distance_threshold = args.distance_threshold
        self.sensor_param.sensor_angle = args.sensor_angle
        self.sensor_param.nsectors = args.nsectors
        self.sensor_param.num_points_on_walls = args.num_points_on_walls
        self.sensor_param.num_points_on_obstacles = args.num_points_on_obstacles
        self.sensor_param.narrow_angle = np.deg2rad(args.narrow_angle)
        self.sensor_param.angle_inbetween = np.deg2rad(args.angle_inbetween)
        self.sensor_param.resolution = np.deg2rad(args.resolution)
        self.sensor_param.Sr = args.Sr

        # Update reward_param attributes
        self.reward_param.alpha_lambda = args.alpha_lambda
        self.reward_param.beta_lambda = args.beta_lambda
        self.reward_param.gamma_theta = args.gamma_theta
        self.reward_param.gamma_x = args.gamma_x
        self.reward_param.gamma_e = args.gamma_e
        self.reward_param.alpha_r = args.alpha_r
        self.reward_param.r_collision = args.r_collision
        self.reward_param.r_arrived = args.r_arrived
        self.reward_param.r_terminated = args.r_terminated
        self.reward_param.max_total_reward = args.max_total_reward
        self.reward_param.lagging_frac = args.lagging_frac
        self.reward_param.r_oa_frac = args.r_oa_frac

        # Update env_param attributes
        self.full_plot = args.full_plot
        self.rangefinder = args.rangefinder
        self.update()
    
    def check_param(self):
        self.print_param()
        assert self.path_param.target_finishing_time >= self.path_param.Lp / self.atr_param.atr_linear_vel_max, "The target finishing time is too short."