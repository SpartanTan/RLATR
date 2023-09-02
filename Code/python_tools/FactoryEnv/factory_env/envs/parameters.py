
import numpy as np

class path_param:
    def __init__(self) -> None:
        self.trajectory_point_interval=0.1 # the distance between each two points on the path
        self.No=8        # number of waypoints
        self.Nw=8        # number of obstacles
        self.Lp=6        # desired length of the path
        self.mu_r=0.25   # mean of the raidus of the obstacle
        self.sigma_d=0.5 # standard deviation of the distance between the obstacle and the path
        self.shift_distance=1 # distance between the path and the walls
        self.extend_length=2.0 # how much should the wall extended at the two ends of the path
        self.look_ahead_distance=0.5 # how far should the robot look ahead
        self.target_finishing_time=30 # how long should the robot finish the path [s], used for setting ghost trajectory
        self.without_obstacles = False # Generate a path without obstacles
        self.how_many_points_forward = 2 # how many points should be considered for the path following reward

class reward_param:
    def __init__(self) -> None:
        self.alpha_lambda = 1.0 # trade-off paramter
        self.beta_lambda  = 2.0# trade-off paramter
        self.gamma_theta = 4.0 # obstacle avoidance reward parameter
        self.gamma_x = 0.1 # obstacle avoidance reward parameter
        self.gammma_e = 0.5 # path following reward parameter
        self.alpha_r = 0.1 # existance reward parameter
        self.r_collision = -20000
        self.r_arrived = 20000
        self.r_terminated = -20000
        self.max_total_reward = 10000
        self.lagging_frac = 3
        self.r_oa_frac = 5
        
class ATR_param:
    def __init__(self):
        self.dt = 0.1 # time step
        self.wheel_radius=0.125
        self.track_width=0.48
        self.atr_linear_vel_max = 0.2 # m/s
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

class env_param:
    def __init__(self):
        self.path_param = path_param()
        self.reward_param = reward_param()
        self.atr_param = ATR_param()
        self.sensor_param = sensor_param()

        self.max_episode_steps = self.path_param.target_finishing_time / self.atr_param.dt * 3
        
        self.full_plot = True # True if want plotting the dashboad; False for just plotting the trajectory
        self.rangefinder = True # True if want to use rangefinder; False for using the sector-based sensor
        if not self.path_param.without_obstacles:
            self.observation_space_n = 27
        else:
            self.observation_space_n = 6
            