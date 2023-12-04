import numpy as np
import gymnasium as gym
from gymnasium import spaces
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.axes import Axes

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.resolve()))

from factory_env.map.Nona2Grid import Nona2Grid

from factory_env.robot.ATR_RK4 import ATR_RK4
from factory_env.map.atr_path import ATRPath
from factory_env.map.Nona2Polygon import Nona2Polygon

from factory_env.envs.parameters import env_param
from factory_env.robot.RF import RangeFinder


class FactoryEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4, 
                "path_name": ["OnePath", "testPath"]}
    
    def __init__(self,
                 params: env_param,
                 map_dir:str, 
                 path_dir: str, 
                 path_name: str,
                 render_mode=None):
              
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        assert path_name in self.metadata["path_name"]
        self.render_mode = render_mode
        self.params: env_param = params
        self.rf = RangeFinder(self.params.sensor_param, self.params.atr_param.track_width)
        self.atr: ATR_RK4 = ATR_RK4(self.params.atr_param)
        self.fig = None
        self.dynamic_plot = None
        """
 
           ____   _  _____ _   _    ____ ___  _   _ _____ ___ ____         
          |  _ \ / \|_   _| | | |  / ___/ _ \| \ | |  ___|_ _/ ___|        
          | |_) / _ \ | | | |_| | | |  | | | |  \| | |_   | | |  _         
          |  __/ ___ \| | |  _  | | |__| |_| | |\  |  _|  | | |_| |        
          |_| /_/   \_\_| |_| |_|  \____\___/|_| \_|_|   |___\____|        
                                                                           
 
        """
        self.map_dir = map_dir
        self.path_dir = path_dir
        self.path_name = path_name
        # self.grid = Nona2Grid(map_dir, 0.2) # ndarrary 40 * 65, upside down
        self.path = ATRPath(params.path_param, path_dir=path_dir, path_name=path_name, atr_trackwidth=self.params.atr_param.track_width, ghost_max_vel=self.params.atr_param.atr_linear_vel_max, if_render_text=False, path_type='pchip')
        """
 
           __  __    _    ____     ____ ___  _   _ _____ ___ ____ 
          |  \/  |  / \  |  _ \   / ___/ _ \| \ | |  ___|_ _/ ___|
          | |\/| | / _ \ | |_) | | |  | | | |  \| | |_   | | |  _ 
          | |  | |/ ___ \|  __/  | |__| |_| | |\  |  _|  | | |_| |
          |_|  |_/_/   \_\_|      \____\___/|_| \_|_|   |___\____|
                                                                  
 
        """
        self.polygon_map = Nona2Polygon(map_dir)
        x_min, x_max, y_min, y_max = self.polygon_map.find_boundary()
        heading_min = -np.pi
        heading_max = np.pi
        
        """
 
              _  _____ ____    ____ ___ __  __ _   _ _        _  _____ ___ ___  _   _ 
             / \|_   _|  _ \  / ___|_ _|  \/  | | | | |      / \|_   _|_ _/ _ \| \ | |
            / _ \ | | | |_) | \___ \| || |\/| | | | | |     / _ \ | |  | | | | |  \| |
           / ___ \| | |  _ <   ___) | || |  | | |_| | |___ / ___ \| |  | | |_| | |\  |
          /_/   \_\_| |_| \_\ |____/___|_|  |_|\___/|_____/_/   \_\_| |___\___/|_| \_|
                                                                                      
 
        """ 
        self.start = self.path.waypoints[0].copy()
        self.goal = self.path.waypoints[-1].copy() # numpy array, (2,)
        self.wheel_radius = 0.125
        self.track_width = 0.48
        self.time_step = 0.05
        
        '''
           ___                   _             
          | __|_ ___ __  ___ ___| |_ _  _ _ __ 
          | _|| ' \ V / (_-</ -_)  _| || | '_ \
          |___|_||_\_/  /__/\___|\__|\_,_| .__/
                                         |_|   
        '''

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.params.observation_space_n,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=self.params.atr_param.atr_w_min, high=self.params.atr_param.atr_w_max, shape=(2,), dtype=np.float32)
        
        self.ep_step = 0
    
    def step(self, action:np.ndarray)->tuple:
        """
        `step` method contains most of the logic of the environment. 
        Maps the action (element of {0,1,2,3}) to the direction we walk in the gridworld

        ### Parameters
        - `action`: np.ndarray, e.g. np.array([2.0, 4.0])
        Notice that the sequence is wheel speed [right, left]

        ### Return
        - a tuple, (observation: dict, reward: float, done: bool, info: dict)

        ### Examples
        >>> self.step([2.0, 4.0]) # agent should move right on grid
        """
        self.ep_step += 1
        if self.params.path_param.dynamic_obstacles:
            self.path.update_obstacles()
        self.atr.runge_kutta_4_step(action, method="simple")
        self.eta, self.index, self.look_ahead_point, _, self.look_ahead_course_error, self.course_error, self.distance_to_goal = self.path.calculate_error_vector(self.atr.state)
        self.intersections_, self.obstacles_obs, self.measurements, self.results, self.closeness = self.rf.closeness_cal(self.atr.state, self.path.obstacles_np)

        if self.params.path_param.without_anything:
            self.Lambda = 1.0
            self.too_close = False
        else:
            if self.obstacles_obs.shape[0] == 0:
                self.Lambda = 1.0
                self.too_close = False
            else:
                min_distance_index = np.argmin(self.obstacles_obs[:, 0])
                min_distance = self.obstacles_obs[min_distance_index, 0]
                corresponding_angle = self.obstacles_obs[min_distance_index, 1]
                avg_distance = np.mean(self.obstacles_obs[:, 0])
                avg_angle = abs(np.mean(self.obstacles_obs[:, 1]))
                gamma_lambda = 0.8
                Lambda_values = np.tanh(gamma_lambda * ((self.obstacles_obs[:, 0] + 1) * 
                  (np.sin(np.minimum(np.abs(self.obstacles_obs[:, 1]), np.pi / 2)) + 1) - 1))
                min_Lambda_index = np.argmin(Lambda_values)
                self.Lambda = Lambda_values[min_Lambda_index]

                # self.Lambda = np.clip(self.Lambda, 0.0, 1.0)
                self.Lambda = np.clip(np.round(self.Lambda, 2), 0.0, 1.0)
                if min_distance < 0.2:
                    self.too_close = True
                    self.Lambda = 0.1
                else:
                    self.too_close = False

        hit_obstacle = self.path.is_atr_in_obstacles(self.atr.state[0:2])
        hit_wall = self.polygon_map.if_collision(self.atr.state[0:2])
        if self.index == len(self.path.even_trajectory)-1:
            is_arrived = self.path.is_arrived(self.atr.state[0:2])
        else:
            is_arrived = False

        if hit_obstacle or hit_wall:
            print("!!HIT")
            self.terminated = True

        if is_arrived:
            print("!!ARRIVED")
            self.terminated = True
        reward_ = 0.0
        observation_ = self._get_obs()
        info_ = self._get_info(is_arrived)
        return observation_, reward_, self.terminated, False, info_

    def _get_obs(self):
        """
        Compute the observation from states. May be used in `reset()` and `step()`.
        Will return a dictionary, contains the agent's and the target's location.

        ### Return
        - a dictionary contains the agent's and the target's location.

        ### Examples
        >>> agent_location = self._get_obs()["agent"] 
        (1,2)
        >>> target_location = self._get_obs()["target"]
        (3,4)
        """
        p = np.array([self.atr.linear_velocity, 
                                self.atr.angular_velocity, 
                                self.look_ahead_course_error, 
                                self.course_error, 
                                self.eta[0],
                                self.eta[1],
                                np.log10(self.Lambda)])
        p = np.concatenate([p, self.closeness])
        return p.astype(np.float32)
    
    def _get_info(self, is_arrived):
        return {
        "steps": self.ep_step,
        'is_arrived': is_arrived,
        'finish_rate': np.round(100 * (self.index / len(self.path.even_trajectory)),2),
        }
    
    def reset(self, seed=None, options=None)->tuple:
        super().reset(seed=seed)
        init_state = np.array(
            [self.path.waypoints[0][0], self.path.waypoints[0][1], self.path.yaw_angles[0]])
        # test
        # init_state = [4.02, 3.57, 0]
        self.ep_step = 0
        self.Lambda = 1.0
        self.reward_trade_off = np.log10(self.Lambda)

        self.atr.reset(init_state)
        self.rf.reset(self.polygon_map.walls, 'real')
        is_arrived = False

        self.closest_point_on_wall = np.array([0.0, 0.0], dtype=float)
        self.closest_distance_to_wall = 0.0
        self.danger = False
        self.terminated = False
        self.trajectory_ = [init_state]

        self.eta, self.index, self.look_ahead_point, look_ahead_point_yaw, self.look_ahead_course_error, self.course_error, self.distance_to_goal = self.path.calculate_error_vector(self.atr.state)
        self.intersections_, self.obstacles_obs, self.measurements, self.results, self.closeness = self.rf.closeness_cal(self.atr.state, self.path.obstacles_np)

        observation_ = self._get_obs()
        info_ = self._get_info(is_arrived)

        return observation_, info_

    def check_collision(self, state)->bool:
        """
        #### Parameters
        - `location`: a list, contains the location of the agent in real coordinate frame
        """
        ifcolli = self.polygon_map.check_if_collision(state[:2])
        return ifcolli


    # def render(self):
    #     def draw_arrow(ax: Axes, x, y, angle, length=1, color='b', alpha=1.0):
    #         dx = length * np.cos(angle)
    #         dy = length * np.sin(angle)
    #         ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc=color, ec='black', alpha=alpha)

    #     if self.render_mode == "human":
    #         fig, ax = plt.subplots()
    #         self.polygon_map.render(ax)
    #         self.path.render()
            
    #         # plot the atr position
    #         plt.scatter(self.atr.state[0], self.atr.state[1], s=100, marker='x', color='red')

    #         for idx, angle in enumerate(self.params.sensor_param.beams):
    #                         angle += self.atr.state[2]
    #                         draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], angle, self.measurements[idx][1], alpha=0.3)
    #             # draw detected points on the obstacles and walls
    #             self.ax0.scatter([x for x, y in self.intersections_], [y for x, y in self.intersections_], color='g', alpha=0.3)
            
    #         plt.axis("equal")
    #         plt.show()
    #     else:
    #         print("Render mode is not 'human', no rendering.")

    def render(self):
        def draw_arrow(ax: Axes, x, y, angle, length=1, color='b', alpha=1.0):
            dx = length * np.cos(angle)
            dy = length * np.sin(angle)
            ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc=color, ec='black', alpha=alpha)

        if self.params.full_plot:
            if self.fig is None:
                self.fig = plt.figure(figsize=(8, 6))
                gs = self.fig.add_gridspec(2, 2)
                self.ax0 = self.fig.add_subplot(gs[:, 0])
                self.polygon_map.render(self.ax0)
                self.path.render(self.ax0)
                # self.ax0.scatter(self.look_ahead_point[0], self.look_ahead_point[1], s=100, marker='*', label='Target Point')
                # draw atr arrow
                draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], self.atr.state[2], 1.0, color='r', alpha=1)
                
                if not self.params.path_param.without_anything:
                    if not self.params.rangefinder:
                        # draw closest points on walls
                        for i in range(self.params.sensor_param.num_points_on_walls):
                            self.ax0.scatter(self.closest_point_on_walls[i][0], self.closest_point_on_walls[i]
                                    [1], s=100, marker='x')
                        # draw arrow to obstacle points on each obstacle
                        for i, obs in enumerate(self.obstacles_obs):
                            draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], obs[2], obs[0], alpha=0.3)
                    else: # using rangefinder model
                        # draw beams with length of the distance to object
                        for idx, angle in enumerate(self.params.sensor_param.beams):
                            angle += self.atr.state[2]
                            draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], angle, self.measurements[idx][1], alpha=0.3)
                        # draw detected points on the obstacles and walls
                        self.ax0.scatter([x for x, y in self.intersections_], [y for x, y in self.intersections_], color='g', alpha=0.3)

                self.ax0.legend()
                self.ax0.set_xlabel("x")
                self.ax0.set_ylabel("y")
                self.ax0.set_title("Trajectory and Obstacles and boundary")
                self.ax0.axis("equal")
                self.ax0.grid()
                
                # Plot sectors
                self.ax1 = self.fig.add_subplot(gs[0, 1])
                for angle in self.params.sensor_param.sectors:
                    draw_arrow(self.ax1, 0.0, 0.0, angle, 1.0)        
                # for data in self.mes:
                #     draw_arrow(self.ax1, 0.0, 0.0, data[0], data[1], 'r')
                if not self.params.path_param.without_anything:
                    for idx, angle in enumerate(self.params.sensor_param.angles_of_sectors):
                        angle += self.atr.state[2]
                        draw_arrow(self.ax1, 0.0, 0.0, angle, self.results[idx], color='r') 
                self.ax1.set_title("Fake renge sensor data")
                self.ax1.axis("equal")
            
                
                self.ax2 = self.fig.add_subplot(gs[1, 1])
                self.ax2.bar(np.rad2deg(self.params.sensor_param.angles_of_sectors), self.closeness)
                self.ax2.set_ylim([0.0, 1.2]) 
                self.ax2.set_title("Closeness")
                
                self.fig.tight_layout()
            
            # Refresh plotting
            # Left side main plot
            self.ax0.clear()
            self.polygon_map.render(self.ax0)
            self.path.render(self.ax0)
            # self.ax0.scatter(self.look_ahead_point[0], self.look_ahead_point[1], s=100, marker='*', label='Target Point')
            # Plot the ATR arrow
            self.ax0.add_patch(plt.Circle((self.atr.state[0], self.atr.state[1]), self.params.atr_param.track_width/2, color='r', fill=False))
            draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], self.atr.state[2], 1.0, color='r', alpha=1.0)
            if not self.params.path_param.without_anything:
                if not self.params.rangefinder:
                    # draw closest points on walls
                    for i in range(self.params.sensor_param.num_points_on_walls):
                        self.ax0.scatter(self.closest_point_on_walls[i][0], self.closest_point_on_walls[i]
                                [1], s=100, marker='x')
                    # draw arrow to obstacle points on each obstacle
                    for i, obs in enumerate(self.obstacles_obs):
                        draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], obs[2], obs[0], alpha=0.3)
                else: # using rangefinder model
                    # draw beams with length of the distance to object
                    for idx, angle in enumerate(self.params.sensor_param.beams):
                        angle += self.atr.state[2]
                        draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], angle, self.measurements[idx][1], alpha=0.3)
                    # draw detected points on the obstacles and walls
                    self.ax0.scatter([x for x, y in self.intersections_], [y for x, y in self.intersections_], color='g', alpha=0.3)
            
            # # refresh ghost ATR    
            # if self.dynamic_plot is not None:
            #     self.dynamic_plot.remove()
            # self.dynamic_plot = self.ax0.scatter(self.ghost_atr_position[0], self.ghost_atr_position[1], color='red', s=20, marker='x', label="Ghost ATR")
            
            self.ax0.legend()
            self.ax0.set_xlabel("x")
            self.ax0.set_ylabel("y")
            self.ax0.set_title("Trajectory and Obstacles and boundary")
            self.ax0.axis("equal")
            self.ax0.grid()
            # self.ax0.text(-1.0, -1.0, f"[wr, wl]: {self.action}\n vel: {self.atr.linear_velocity:.2f}\n omega: {self.atr.angular_velocity:.2f}\n look_ahead_course_error: {self.look_ahead_course_error:.2f}\n course_error: {self.course_error:.2f}\n cross_track_error: {self.eta[1]:.2f}\n reward_trade_off: {self.reward_trade_off:.2f}\n reward_pf: {self.reward_pf:.2f}\n reward_oa: {self.reward_oa:.2f}\n reward: {self.reward:.2f}\n total_r: {self.total_reward:.2f}\n terminated: {self.terminated}")
            # self.ax0.annotate(f"[wr, wl]: {self.action}\n vel: {self.atr.linear_velocity:.2f}\n omega: {self.atr.angular_velocity:.2f}\n look_ahead_course_error: {self.look_ahead_course_error:.2f}\n course_error: {self.course_error:.2f}\n cross_track_error: {self.eta[1]:.2f}\n", xy=(0, 0), xycoords='axes fraction', ha='left', va='bottom')

            
            # Plot sectors
            self.ax1.clear()
            # draw bound of each sector
            if not self.params.path_param.without_anything:
                for angle in self.params.sensor_param.sectors:
                    angle += self.atr.state[2]
                    draw_arrow(self.ax1, 0.0, 0.0, angle, 1.0)
                for idx, angle in enumerate(self.params.sensor_param.angles_of_sectors):
                    angle += self.atr.state[2]
                    draw_arrow(self.ax1, 0.0, 0.0, angle, self.results[idx], color='r')     
            #     self.ax1.annotate(f"lambda: {self.Lambda}\nr_pf: {self.reward_pf:.2f}\nr_oa: {self.reward_oa:.2f}\nr_speed: {self.r_low_speed:.2f}\nr: {self.reward:.2f}\ntotal_r: {self.total_reward}\nterminated: {self.terminated}", xy=(0, 0), xycoords='axes fraction', ha='left', va='bottom')
            # else:
            #     self.ax1.annotate(f"lambda: {self.Lambda}\nr_pf: {self.reward_pf:.2f}\nr_oa: {self.reward_oa:.2f}\nr_speed: {self.r_low_speed:.2f}\nr: {self.reward:.2f}\ntotal_r: {self.total_reward}\nterminated: {self.terminated}", xy=(0, 0), xycoords='axes fraction', ha='left', va='bottom')
                       
            self.ax1.set_title("Fake renge sensor data")
            self.ax1.axis("equal")
            
            # Plot closeness bar
            self.ax2.clear()
            self.ax2.bar(np.rad2deg(self.params.sensor_param.angles_of_sectors), self.closeness)
            self.ax2.set_ylim([0.0, 1.2]) 
            self.ax2.set_title("Closeness")
            
            # Convert the figure to a numpy array
            # self.fig.canvas.draw()
            # data = np.frombuffer(self.fig.canvas.buffer_rgba(), dtype=np.uint8)
            # data = data.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            # return data
        
        else:
            if self.fig is None:
                self.fig, self.ax0 = plt.subplots()
                # gs = self.fig.add_gridspec(2, 2)
                # self.ax0 = self.fig.add_subplot(gs[:, 0])
                self.polygon_map.render(self.ax0)
                self.path.render(self.ax0)
                # self.ax0.scatter(self.look_ahead_point[0], self.look_ahead_point[1], s=100, marker='*', label='Target Point')
                # draw atr arrow
                self.ax0.add_patch(plt.Circle((self.atr.state[0], self.atr.state[1]), self.params.atr_param.track_width/2, color='r', fill=False))
                draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], self.atr.state[2], 1.0, color='r', alpha=1)
                if not self.params.path_param.without_anything:
                    if not self.params.rangefinder:
                        # draw closest points on walls
                        for i in range(self.params.sensor_param.num_points_on_walls):
                            self.ax0.scatter(self.closest_point_on_walls[i][0], self.closest_point_on_walls[i][1], s=100, marker='x')
                        # draw arrow to obstacle points on each obstacle
                        for i, obs in enumerate(self.obstacles_obs):
                            draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], obs[2], obs[0], alpha=0.3)
                    else: # using rangefinder model
                        # draw beams with length of the distance to object
                        for idx, angle in enumerate(self.params.sensor_param.beams):
                            angle += self.atr.state[2]
                            draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], angle, self.measurements[idx][1], alpha=0.3)
                        # draw detected points on the obstacles and walls
                        self.ax0.scatter([x for x, y in self.intersections_], [y for x, y in self.intersections_], color='g', alpha=0.3)

                self.ax0.legend()
                self.ax0.set_xlabel("x")
                self.ax0.set_ylabel("y")
                self.ax0.set_title("Trajectory and Obstacles and boundary")
                self.ax0.axis("equal")
                self.ax0.grid()

            # Refresh plotting
            # Left side main plot
            self.ax0.clear()
            self.polygon_map.render(self.ax0)
            self.path.render(self.ax0)
            # self.ax0.scatter(self.look_ahead_point[0], self.look_ahead_point[1], s=100, marker='*', label='Target Point')
            # Plot the ATR arrow
            self.ax0.add_patch(plt.Circle((self.atr.state[0], self.atr.state[1]), self.params.atr_param.track_width/2, color='r', fill=False))
            draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], self.atr.state[2], 1.0, color='r', alpha=1.0)
            if not self.params.path_param.without_anything:
                if not self.params.rangefinder:
                    # draw closest points on walls
                    for i in range(self.params.sensor_param.num_points_on_walls):
                        self.ax0.scatter(self.closest_point_on_walls[i][0], self.closest_point_on_walls[i]
                                [1], s=100, marker='x')
                    # draw arrow to obstacle points on each obstacle
                    for i, obs in enumerate(self.obstacles_obs):
                        draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], obs[2], obs[0], alpha=0.3)
                else: # using rangefinder model
                    # draw beams with length of the distance to object
                    for idx, angle in enumerate(self.params.sensor_param.beams):
                        angle += self.atr.state[2]
                        draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], angle, self.measurements[idx][1], alpha=0.3)
                    # draw detected points on the obstacles and walls
                    self.ax0.scatter([x for x, y in self.intersections_], [y for x, y in self.intersections_], color='g', alpha=0.3)
            
            # # refresh ghost ATR    
            # if self.dynamic_plot is not None:
            #     self.dynamic_plot.remove()
            # self.dynamic_plot = self.ax0.scatter(self.ghost_atr_position[0], self.ghost_atr_position[1], color='red', s=20, marker='x', label="Ghost ATR")
            
            # self.ax0.annotate(f"[wr, wl]: {self.action}\n" \
            #       f"vel: {self.atr.linear_velocity:.2f}\n" \
            #       f"omega: {self.atr.angular_velocity:.2f}\n" \
            #     #   f"look_ahead_course_error: {np.rad2deg(self.look_ahead_course_error):.2f}\u00b0\n" \
            #       f"course_error: {np.rad2deg(self.course_error):.2f}\u00b0\n" \
            #       f"cross_track_error: {self.eta[1]:.2f}\n" \
            #       f"\n" \
            #       f"lambda: {self.Lambda}\n" \
            #       f"r_pf: {self.reward_pf:.2f}\n" \
            #       f"r_oa: {self.reward_oa:.2f}\n" \
            #       f"r_speed: {self.r_low_speed}\n" \
            #       f"r: {self.reward:.2f}\n" \
            #       f"total_r: {self.total_reward}\n" \
            #       f"terminated: {self.terminated}\n", \
            #       xy=(1.2, 1.0), xycoords='axes fraction', ha='left', va='top')
            # obs = self._get_obs()
            # self.ax0.annotate(f"next_obs: \n" \
            #                   f"[1] linear velocity:{obs[0]}\n" \
            #                   f"[2] angular velocity:{obs[1]}\n" \
            #                   f"[3] look_ahead_course_error:{obs[2]}\n" \
            #                   f"[4] course_error:{obs[3]}\n" \
            #                   f"[5] cross_track_error:{obs[4]}\n", \
            #                 xy=(1.2, 0.5), xycoords='axes fraction', ha='left', va='top')
                
            # # Convert the figure to a numpy array
            # self.fig.canvas.draw()
            # data = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
            # data = data.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            # return data
        

    def print_demo_helper_info(self):
        example_observation = {'agent': np.array([ 0.679,  5.09 , -0.026]), 
                               'target': np.array([10.31,  3.68])}
        example_info = {'trajectory': np.array([[ 0.66 ,  5.09 ,  0.   ],
                                             [ 0.679,  5.09 , -0.026]]),
                        'distance': 9.733880470067657}
        print(f"Example action: np.array([2.0, 4.0])")
        print(f"Example observation: {example_observation}")
        print(f"Example info: {example_info}")
        print(f"Shape of trajectory: (num_of_steps, 2)")
            
if __name__ == "__main__":
    map_dir = "/home/zhicun/code/atr_rl/Code/python_tools/FactoryEnv/factory_env/map/config/nona_description.json"
    path_dir = "/home/zhicun/code/atr_rl/Code/python_tools/FactoryEnv/factory_env/map/config/nodes_tuve.json"
    path_name = "OnePath"
    params:env_param = env_param()
    env = FactoryEnv(params, map_dir, path_dir, path_name, render_mode="human")
    observation, info = env.reset()
    env.step(np.array([0.0, 0.0]))
    # env.SIM_ATR.set_position(np.array([10.5, 3]))
    position = env._get_obs()["agent"]
    print(position)
    
    # for i in range(100):
    #     action = np.array([2.0, 2.0])
    #     observation, reward, terminated, truncated, info = env.step(action)
    # print(observation)
    # print(info)
    # observation, info  = env.reset()
    env.render()
    plt.show()
    # env.print_demo_helper_info()
    # action = env.action_space.sample()
    # action = np.array([2.0, 4.0])
    # action = np.array([2.0, 4.0])
    
    # observation, reward, terminated, truncated, info = env.step(action)
    # print(observation)
    # print(info)

    # action_traj = []
    # time_step = 0.05
    # simulation_duration = 50  # in seconds
    # n_steps = int(simulation_duration / time_step)
    # for step in range(n_steps):
    #     action = env.action_space.sample()
    #     observation, reward, terminated, truncated, info  = env.step(action)
    #     action_traj.append(np.array(action))
    # trajectory = env._get_info()["trajectory"]
    # action_traj = np.array(action_traj)
    # plt.plot(trajectory[:, 0], trajectory[:, 1], 'r--', linewidth = 1)
    # print(f'final state: {trajectory[-1,0], trajectory[-1, 1], trajectory[-1, 2]}')
    
    # env.render()
    
    # fig = plt.figure()
    # plt.plot(action_traj[:, 0], 'r--', linewidth = 1)
    # plt.plot(action_traj[:, 1], 'b--', linewidth = 1)
    # plt.show()
    # env.step(3)
    # observation = env._get_obs()
    # print(observation)
    