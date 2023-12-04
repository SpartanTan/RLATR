import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from typing import Optional, Tuple, Union

import time
import pickle
import math
import gymnasium as gym
from gymnasium import spaces

from factory_env.map.env_path import Path
from factory_env.robot.ATR_RK4 import ATR_RK4
from factory_env.robot.RF import RangeFinder

from factory_env.envs.parameters import env_param

import pkg_resources
import os


class SimFactoryEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self, params: env_param, render_mode: Optional[str] = None):

        self.params: env_param = params
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.path: Path = Path(self.params.path_param, self.params.atr_param.atr_linear_vel_max, self.params.atr_param.track_width)
        self.atr: ATR_RK4 = ATR_RK4(self.params.atr_param)
        self.rf = RangeFinder(self.params.sensor_param, self.params.atr_param.track_width)

        # action: [wr, wl]
        self.action_space = spaces.Box(
            low=self.params.atr_param.atr_w_min, high=self.params.atr_param.atr_w_max, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.params.observation_space_n,), dtype=np.float32)

        self.ghost_index = 0
        
        self.fig = None
        self.dynamic_plot = None
        self.total_reward = 0.0
        self.ep_step = 0
        self.recent_speeds = []
        self.recent_angular_velocities = []
        self.low_speed_duration = 0

    '''
       ___ _____ ___ ___ 
      / __|_   _| __| _ \
      \__ \ | | | _||  _/
      |___/ |_| |___|_|  
                         
    '''
    def step(self, action:np.ndarray):
        self.ep_step += 1
        """
        In this method, ghost ATR will move one index forward. The ATR will move according to the action. 
        ### Parameters
        - `action` np.ndarray, (2,) array, [wr, wl]
        
        ### Returns
        observation, reward, done, truncated, info
        observation: (vel, omega, look_ahead_course_error, course_error, cross_track_error)
        """
        if self.params.path_param.dynamic_obstacles:
            self.path.update_obstacles()
        # Ghost ATR step
        self.ghost_index += 1
        if self.ghost_index >= len(self.path.ghost_trajectory):
            self.ghost_index = len(self.path.ghost_trajectory) - 1
        self.ghost_atr_position = self.path.ghost_trajectory[self.ghost_index]
        
        # ATR step
        self.action = np.clip(action, self.params.atr_param.atr_w_min, self.params.atr_param.atr_w_max) # make sure action is with boundary
        self.atr.runge_kutta_4_step(self.action, method="simple")
        # self.atr.state = np.array([self.ghost_atr_position[0], self.ghost_atr_position[1], self.path.ghost_yaw_angles[self.ghost_index]])
        self.eta, self.index, self.look_ahead_point, _, self.look_ahead_course_error, self.course_error, self.distance_to_goal = self.path.calculate_error_vector(self.atr.state)
        
        # do not use sensor if there are nothing
        if not self.params.path_param.without_anything:
            # my dirty method
            if not self.params.rangefinder: 
                if not self.params.path_param.without_walls:
                    self.min_distance_to_walls, self.closest_point_on_walls = self.path.minimum_distance_to_walls(
                    self.atr.state[0:2], method='kdtree', num_of_closest_points=self.params.sensor_param.num_points_on_walls)
                self.closeness, self.obstacles_obs, self.results = self.closeness_cal()
            # current method
            else:
                self.intersections_, self.obstacles_obs, self.measurements, self.results, self.closeness = self.rf.closeness_cal(self.atr.state, self.path.obstacles_np)    
            hit_obstacle = self.path.is_atr_in_obstacles(self.atr.state[0:2])
            if not self.params.path_param.without_walls:
                hit_wall = not self.path.is_inside(self.atr.state[0:2])
            else:
                hit_wall = False
        else:
            hit_obstacle = False
            hit_wall = False
            self.obstacles_obs = np.array([])
            self.closeness = np.zeros(self.params.sensor_param.nsectors)
        
        if self.index == len(self.path.even_trajectory)-1:
            is_arrived = self.path.is_arrived(self.atr.state[0:2], 2)
        else:
            is_arrived = False
        # if the distance to the nearest obstacle is too close, turn lambda into 0
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
                gamma_lambda = 0.6
                Lambda_values = np.tanh(gamma_lambda * ((self.obstacles_obs[:, 0] + 1) * 
                  (np.sin(np.minimum(np.abs(self.obstacles_obs[:, 1]), np.pi / 2)) + 1) - 1))
                min_distance = self.obstacles_obs[min_distance_index, 0]
                min_Lambda_index = np.argmin(Lambda_values)
                self.Lambda = Lambda_values[min_Lambda_index]

                # self.Lambda = np.clip(self.Lambda, 0.0, 1.0)
                self.Lambda = np.clip(np.round(self.Lambda, 2), 0.0, 1.0)
                if min_distance < self.params.critical_distance:
                    self.too_close = True
                    self.Lambda = 0.1
                else:
                    self.too_close = False
                #     self.Lambda = self.Lambda_bk
        self.reward_pf, self.reward_oa, self.r_low_speed, self.reward = self.reward_calc()
        self.terminated = False
        if hit_obstacle or hit_wall:
            print("!!HIT")
            self.once_hit = True
            self.terminated = True
            self.reward = (1-self.Lambda) * self.params.reward_param.r_collision
        if is_arrived:
            if self.once_hit:
                print("!!HIT AND ARRIVED")
            else:
                print("!!ARRIVED")
            print(f"Total steps: {self.ep_step}")
            print(f"Finishing rate: {np.round(100 * (self.index / len(self.path.even_trajectory)),2)}")
            self.terminated = True
            step_penalty_rate = 10
            step_penalty = step_penalty_rate * (np.exp((self.ep_step - self.optimal_total_steps) / self.arrive_reward_adj) - 1) if self.ep_step > self.optimal_total_steps else 0
            self.reward = self.params.reward_param.r_arrived - step_penalty 
        # terminate this episode if the total reward is too large
        self.total_reward += self.reward
        if abs(self.total_reward) > self.params.reward_param.max_total_reward:
            print("REWAED TOO LARGE")
            self.terminated = True
            self.reward = self.params.reward_param.r_terminated
            finishin_rate = np.round(100 * (self.index / len(self.path.even_trajectory)),2)
            self.reward += finishin_rate * 2
            print(f"Finishing rate: {finishin_rate}")

        if self.ep_step > self.params.path_param.max_ep_steps:
            print("STEPS TOO LARGE")
            self.terminated = True
            self.reward = self.params.reward_param.r_terminated
            finishin_rate = np.round(100 * (self.index / len(self.path.even_trajectory)),2)
            self.reward += finishin_rate * 2
            print(f"Finishing rate: {finishin_rate}")
            
        # observations for the agent
        next_obs = self._get_obs()
        info = self._get_info(is_arrived)
        # info = {}
        truncated = False
        return next_obs, self.reward, self.terminated, truncated, info
    
    '''
        ___ ___ _____    ___  ___ ___ 
       / __| __|_   _|  / _ \| _ ) __|
      | (_ | _|  | |   | (_) | _ \__ \
       \___|___| |_|    \___/|___/___/
                                      
    '''
    def _get_obs(self):
        """
        Get the observation for the agent.
        
        ### Returns
        (26, ) numpy array, [vel, omega, look_ahead_course_error, course_error, cross_track_error, reward_trade_off, closeness(20,)]
        """
        if not self.params.path_param.without_anything:
            p = np.array([self.atr.linear_velocity, 
                                self.atr.angular_velocity, 
                                self.look_ahead_course_error, 
                                self.course_error, 
                                self.eta[0],
                                self.eta[1], 
                                self.Lambda])
            p = np.concatenate([p, self.closeness])
        else:
            p = np.array([self.atr.linear_velocity, 
                                self.atr.angular_velocity, 
                                self.look_ahead_course_error, 
                                self.course_error, 
                                self.eta[0],
                                self.eta[1],
                                self.Lambda])
            p = np.concatenate([p, self.closeness])
        return p.astype(np.float32)
    
    def _get_info(self, is_arrived):
        return {
        "steps": self.ep_step,
        'is_arrived': is_arrived,
        'finish_rate': np.round(100 * (self.index / len(self.path.even_trajectory)),2),
        }

    '''
       ___ _____      ___   ___ ___  
      | _ \ __\ \    / /_\ | _ \   \ 
      |   / _| \ \/\/ / _ \|   / |) |
      |_|_\___| \_/\_/_/ \_\_|_\___/ 
                                     
    '''
    def reward_calc(self):
        r_oa = 0.0
        r_pf = 0.0
        r_lagging = 0.0
        den = 0.0
        num = 0.0
        # Update the list of recent speeds
        self.recent_speeds.append(self.atr.linear_velocity)
        self.recent_angular_velocities.append(self.atr.angular_velocity)
        if len(self.recent_speeds) > self.params.reward_param.vel_window:
            self.recent_speeds.pop(0)  # Remove the oldest speed
            self.recent_angular_velocities.pop(0)
            
        # Calculate the average speed
        # average_speed = sum(self.recent_speeds) / len(self.recent_speeds)
        average_speed = sum(speed for speed in self.recent_speeds) / len(self.recent_speeds)
        average_angular_velocity = sum(angular_velocity for angular_velocity in self.recent_angular_velocities) / len(self.recent_angular_velocities)
        # Determine if the speed is below the threshold
        if abs(average_speed) < self.params.reward_param.low_speed_threshold:
            self.low_speed_duration += self.params.reward_param.duration  # Increment duration of low speed
        else:
            self.low_speed_duration = 0
        # reward for turning hard when getting too close
        if self.too_close:
            r_turn = 10 * abs(average_angular_velocity)
        else:
            r_turn = 0.0
        # Calculate the penalty
        # The penalty increases as the average speed decreases and as the duration of low speed continues
        # speed_factor = max(0, 1 - average_speed / self.params.reward_param.low_speed_threshold)
        speed_factor = max(0, 1 - abs(average_speed) / self.params.reward_param.low_speed_threshold)
        low_speed_penalty = -self.params.reward_param.low_speed_penalty_rate * self.low_speed_duration * speed_factor
        if low_speed_penalty < -self.params.reward_param.max_low_speed_penalty:
            low_speed_penalty = -self.params.reward_param.max_low_speed_penalty
        if self.too_close:
            low_speed_penalty *= 0.2
        # Additional penalty for moving backward
        backward_penalty = 0
        if average_speed < 0:
            backward_penalty = -self.params.reward_param.backward_penalty_rate * abs(average_speed)
        if self.too_close:
            backward_penalty = 0.0
        # reward for getting closer to the goal
        # Check if the robot has progressed to a new point on the trajectory
        if self.index > self.previous_trajectory_index:
            # Reward for moving closer to the goal
            trajectory_progress_reward = self.params.reward_param.trajectory_progress_reward
        else:
            trajectory_progress_reward = 0
        self.previous_trajectory_index = self.index
        if len(self.obstacles_obs) == 0:          
            # r_oa -= (1 + np.abs(self.params.reward_param.gamma_theta * np.pi)) ** -1 * (self.params.reward_param.gamma_x * 5**2) ** -1
            r_oa = 0.0
        else:
            for idx, data in enumerate(self.obstacles_obs):
                # theta_i = np.round(data[1] / self.params.reward_param.angle_resolution) * self.params.reward_param.angle_resolution
                # x_i = np.round(data[0] / self.params.reward_param.distance_resolution) * self.params.reward_param.distance_resolution
                theta_i = data[1]
                x_i = data[0]
                num += np.log(((1 + np.abs(self.params.reward_param.gamma_theta * theta_i)) ** -1) * ((self.params.reward_param.gamma_x * np.maximum(x_i, self.params.reward_param.epsilon_x)**2) ** -1))
                # num += ((1 + np.abs(self.params.reward_param.gamma_theta * theta_i)) ** -1) * ((self.params.reward_param.gamma_x * np.maximum(x_i, self.params.reward_param.epsilon_x)**2) ** -1)
                # den += ( 1 + np.abs(self.params.reward_param.gamma_theta * theta_i)) ** -1
                den += 1
                # r_oa -= np.log((1 + np.abs(gamma_theta * data[1])) ** -1 * (gamma_x * np.maximum(data[0], 0.1)**2) ** -1)
                # r_oa -= (1 + np.abs(gamma_theta * data[1])) ** -1 * (gamma_x * np.maximum(data[0], 0.1)**2) ** -1
                
                # num += ((1 + np.abs(gamma_theta * data[1])) ** -1) * (np.log(np.abs(gamma_x)) + 2 * np.log(np.maximum(data[0], 0.1)))
                # den += (1 + np.abs(gamma_theta * data[1])) ** -1
            # r_oa /= len(obstacles_obs)
            if num == 0.0:
                r_oa = 0.0
            else:
                r_oa = - num/den
                # r_oa = np.round(r_oa / self.params.reward_param.oa_result_resolution) * self.params.reward_param.oa_result_resolution
                # if self.too_close:
                # r_oa *= 2
                if r_oa > 0:
                    r_oa = 0.0
            # r_oa = -np.log(abs(r_oa))
            # r_oa /= self.params.reward_param.r_oa_frac
        # r_pf = -1 + ( (np.sqrt(abs(self.atr.linear_velocity))/self.params.atr_param.atr_linear_vel_max) * np.cos(self.course_error)+1) * (np.exp(-self.params.reward_param.gamma_e * np.abs(self.eta[1]) + 1))
        course_error_discrete = np.round(self.course_error / self.params.reward_param.angle_resolution) * self.params.reward_param.angle_resolution
        cross_track_error_discrete = np.round(np.abs(self.eta[1]) / self.params.reward_param.distance_resolution) * self.params.reward_param.distance_resolution
        r_pf = -1 + (np.abs(self.atr.linear_velocity)/self.params.atr_param.atr_linear_vel_max * np.cos(course_error_discrete)+1) * (np.exp(-self.params.reward_param.gamma_e * np.abs(cross_track_error_discrete)) + 1)
        # r_pf = np.round(r_pf / self.params.reward_param.pf_result_resolution) * self.params.reward_param.pf_result_resolution
        # r_pf *= 3
        r_lagging = - abs(self.eta[0]) * self.params.reward_param.lagging_frac
        if not self.params.path_param.without_anything:
            r_exists = - self.Lambda * (2*self.params.reward_param.alpha_r + 1 )
            reward = self.Lambda * r_pf + (1 - self.Lambda) * r_oa + r_exists + backward_penalty + low_speed_penalty # + r_lagging + trajectory_progress_reward
        else:
            r_exists = -(2*self.params.reward_param.alpha_r + 1 )
            reward = r_pf + r_exists + backward_penalty # + r_lagging + low_speed_penalty # + backward_penalty + trajectory_progress_reward 
        return r_pf, r_oa, low_speed_penalty, reward

    '''
       ___ ___ _  _ ___  ___ ___ 
      | _ \ __| \| |   \| __| _ \
      |   / _|| .` | |) | _||   /
      |_|_\___|_|\_|___/|___|_|_\
                                 
    '''
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
                self.path.render(self.ax0, False, self.params.path_param.without_walls)
                self.ax0.scatter(self.look_ahead_point[0], self.look_ahead_point[1],
                        s=100, marker='*', label='Target Point')
                # draw atr arrow
                draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], self.atr.state[2], 1.0, color='r', alpha=1)
                self.ax0.add_patch(plt.Circle((self.atr.state[0], self.atr.state[1]), self.params.atr_param.track_width/2, color='r', fill=False))
                if not self.params.path_param.without_anything:
                    if not self.params.rangefinder:
                        # draw closest points on walls
                        if not self.params.path_param.without_walls:
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
            self.path.render(self.ax0, False, self.params.path_param.without_walls)
            self.ax0.scatter(self.look_ahead_point[0], self.look_ahead_point[1],
                    s=100, marker='*', label='Target Point')
            # Plot the ATR arrow
            draw_arrow(self.ax0, self.atr.state[0], self.atr.state[1], self.atr.state[2], 1.0, color='r', alpha=1.0)
            self.ax0.add_patch(plt.Circle((self.atr.state[0], self.atr.state[1]), self.params.atr_param.track_width/2, color='r', fill=False))
            if not self.params.path_param.without_anything:
                if not self.params.rangefinder:
                    # draw closest points on walls
                    if not self.params.path_param.without_walls:
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
            
            # refresh ghost ATR    
            if self.dynamic_plot is not None:
                self.dynamic_plot.remove()
            self.dynamic_plot = self.ax0.scatter(self.ghost_atr_position[0], self.ghost_atr_position[1], color='red', s=20, marker='x', label="Ghost ATR")
            
            self.ax0.legend()
            self.ax0.set_xlabel("x")
            self.ax0.set_ylabel("y")
            self.ax0.set_title("Trajectory and Obstacles and boundary")
            self.ax0.axis("equal")
            self.ax0.grid()
            # self.ax0.text(-1.0, -1.0, f"[wr, wl]: {self.action}\n vel: {self.atr.linear_velocity:.2f}\n omega: {self.atr.angular_velocity:.2f}\n look_ahead_course_error: {self.look_ahead_course_error:.2f}\n course_error: {self.course_error:.2f}\n cross_track_error: {self.eta[1]:.2f}\n reward_trade_off: {self.reward_trade_off:.2f}\n reward_pf: {self.reward_pf:.2f}\n reward_oa: {self.reward_oa:.2f}\n reward: {self.reward:.2f}\n total_r: {self.total_reward:.2f}\n terminated: {self.terminated}")
            self.ax0.annotate(f"[wr, wl]: {self.action}\n vel: {self.atr.linear_velocity:.2f}\n omega: {self.atr.angular_velocity:.2f}\n look_ahead_course_error: {self.look_ahead_course_error:.2f}\n course_error: {self.course_error:.2f}\n cross_track_error: {self.eta[1]:.2f}\n", xy=(0, 0), xycoords='axes fraction', ha='left', va='bottom')

            
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
                self.ax1.annotate(f"lambda: {self.Lambda}\nr_pf: {self.reward_pf:.2f}\nr_oa: {self.reward_oa}\nr_speed: {self.r_low_speed:.2f}\nr: {self.reward:.2f}\ntotal_r: {self.total_reward}\nterminated: {self.terminated}", xy=(0, 0), xycoords='axes fraction', ha='left', va='bottom')
            else:
                self.ax1.annotate(f"lambda: {self.Lambda}\nr_pf: {self.reward_pf:.2f}\nr_oa: {self.reward_oa:.2f}\nr_speed: {self.r_low_speed:.2f}\nr: {self.reward:.2f}\ntotal_r: {self.total_reward}\nterminated: {self.terminated}", xy=(0, 0), xycoords='axes fraction', ha='left', va='bottom')
                       
            self.ax1.set_title("Fake renge sensor data")
            self.ax1.axis("equal")
            
            # Plot closeness bar
            self.ax2.clear()
            self.ax2.bar(np.rad2deg(self.params.sensor_param.angles_of_sectors), self.closeness)
            self.ax2.set_ylim([0.0, 1.2]) 
            self.ax2.set_title("Closeness")
            
            # Convert the figure to a numpy array
            self.fig.canvas.draw()
            data = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
            data = data.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            return data
        
        else:
            if self.fig is None:
                self.fig = plt.figure(figsize=(8, 6))
                gs = self.fig.add_gridspec(2, 2)
                self.ax0 = self.fig.add_subplot(gs[:, 0])
                self.path.render(self.ax0, if_yaw_angle=False, no_plotting_walls=self.params.path_param.without_walls)
                self.ax0.scatter(self.look_ahead_point[0], self.look_ahead_point[1],
                        s=100, marker='*', label='Target Point')
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

            # Refresh plotting
            # Left side main plot
            self.ax0.clear()
            self.path.render(self.ax0, if_yaw_angle=False, no_plotting_walls=self.params.path_param.without_walls)
            self.ax0.scatter(self.look_ahead_point[0], self.look_ahead_point[1],
                    s=100, marker='*', label='Target Point')
            # Plot the ATR arrow
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
            
            # refresh ghost ATR    
            if self.dynamic_plot is not None:
                self.dynamic_plot.remove()
            self.dynamic_plot = self.ax0.scatter(self.ghost_atr_position[0], self.ghost_atr_position[1], color='red', s=20, marker='x', label="Ghost ATR")
            
            self.ax0.annotate(f"[wr, wl]: {self.action}\n" \
                  f"vel: {self.atr.linear_velocity:.2f}\n" \
                  f"omega: {self.atr.angular_velocity:.2f}\n" \
                  f"look_ahead_course_error: {np.rad2deg(self.look_ahead_course_error):.2f}\u00b0\n" \
                  f"course_error: {np.rad2deg(self.course_error):.2f}\u00b0\n" \
                  f"cross_track_error: {self.eta[1]:.2f}\n" \
                  f"\n" \
                  f"lambda: {self.Lambda}\n" \
                  f"r_pf: {self.reward_pf:.2f}\n" \
                  f"r_oa: {self.reward_oa:.2f}\n" \
                  f"r_speed: {self.r_low_speed}\n" \
                  f"r: {self.reward:.2f}\n" \
                  f"total_r: {self.total_reward}\n" \
                  f"terminated: {self.terminated}\n", \
                  xy=(1.2, 1.0), xycoords='axes fraction', ha='left', va='top')
            obs = self._get_obs()
            self.ax0.annotate(f"next_obs: \n" \
                              f"[1] linear velocity:{obs[0]}\n" \
                              f"[2] angular velocity:{obs[1]}\n" \
                              f"[3] look_ahead_course_error:{obs[2]}\n" \
                              f"[4] course_error:{obs[3]}\n" \
                              f"[5] cross_track_error:{obs[4]}\n", \
                            xy=(1.2, 0.5), xycoords='axes fraction', ha='left', va='top')
                
            # Convert the figure to a numpy array
            self.fig.canvas.draw()
            data = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
            data = data.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            return data
        
    def file_logic(self, options):
        """
        This method switches between saving or loading the path. Will be called in `self.reset()`.
        ### Parameters
        - `options` dict, options for the environment. \ 
            keys:
            - `init_type` str, 'run', 'save', 'load' \ 
            'run': create a new path but not save \ 
            'save': create and save this path \ 
            'load': load the path
            - `file_name` str, file name for saving or loading the path. \ 
        
        ### Returns
        Return True if needs to run path.reset()
        
        ### Example
        options = {'init_type': 'save', 'file_name': 'test.pkl'}
        self.file_logic(options)
        

        """
        # if options is None:
        #     options={'init_type': 'run'}
        # if options.get('init_type') == 'run':
        #     self.path.reset()
        #     return True
        # package_dir = pkg_resources.get_distribution('FactoryEnv').location
        # save_dir = package_dir + '/factory_env/map/env_paths/'
        # if options.get('file_name') is None:
        #     file_name = 'short.pkl'
        # else:
        #     file_name = options.get('file_name')
        # file_path = os.path.join(save_dir, file_name)
        # if options.get('init_type') == 'save':
        #     self.path.reset()
        #     with open(file_path, "wb") as file:
        #         pickle.dump(self.path, file)
        #         return False
        # elif options.get('init_type') == 'load':
        #     with open(file_path, "rb") as file:
        #         self.path = pickle.load(file)
        #         return False
        if self.params.path_param.static_map == 0:
            self.path.reset()
            return True
        package_dir = pkg_resources.get_distribution('FactoryEnv').location
        save_dir = package_dir + '/factory_env/map/env_paths/'
        file_name = 'test.pkl'
        file_path = os.path.join(save_dir, file_name)
        if self.params.path_param.static_map == 1:
            self.path.reset()
            with open(file_path, "wb") as file:
                pickle.dump(self.path, file)
                return False
        elif self.params.path_param.static_map == 2:
            with open(file_path, "rb") as file:
                self.path = pickle.load(file)
                self.params.path_param = self.path.params
                self.params.path_param.static_map = 2
                return False       
    '''
       ___ ___ ___ ___ _____ 
      | _ \ __/ __| __|_   _|
      |   / _|\__ \ _|  | |  
      |_|_\___|___/___| |_|  
                             
    '''
    def reset(self,
              *,
              seed=None,
              options=None):
        super().reset(seed=seed)
        self.file_logic(options)
        self.ep_step = 0
        self.recent_speeds = []
        self.low_speed_duration = 0
        self.previous_trajectory_index = 0
        self.optimal_total_steps = int(self.params.path_param.Lp / self.params.atr_param.atr_linear_vel_max / self.params.atr_param.dt)
        self.arrive_reward_adj = 400 * (self.params.path_param.Lp/self.params.path_param.Lp_std)
        # Reset the ATR state
        init_state = np.array(
            [self.path.waypoints[0][0], self.path.waypoints[0][1], self.path.yaw_angles[0]])
        # init_state = np.array([1.113, -4.18, -2.255])
        # init_state = np.array([-0.232, -5.801, -5.089]) # single on the left
        # init_state = np.array([-0.089, -5.703, -2.672]) # single on the left, heading obs
        # init_state = np.array([-0.232, -5.801, -0.056]) # sinlge on the right
        # init_state = np.array([1.07,  -4.172, -2.223]) # complex case
        # init_state = np.array([1.343, -4.811, -2.223]) # close to wall with obs
        # init_state = np.array([1.043, -5.374, -1.973]) # close to wall without obs
        # init_state = np.array([ 0.896, -5.575, -2.973]) # close to wall with wrong angle
        
        self.once_hit = False
        self.too_close = False
        
        self.action = np.array([0.0, 0.0])
        self.atr.reset(init_state)
        self.rf.reset(self.path.walls_LinearRing)
        # reset the ghost ATR
        self.ghost_index = 0
        self.ghost_atr_position = self.path.ghost_trajectory[self.ghost_index]
        
        # sample random trade-off parameter
        self.Lambda = self.Lambda_bk = 10**(-np.random.gamma(self.params.reward_param.alpha_lambda, 1.0/self.params.reward_param.beta_lambda))
        self.reward_trade_off = np.log10(self.Lambda)
        
        # Get error vector, look ahead point, look ahead course error, course error
        self.eta, self.index, self.look_ahead_point, look_ahead_point_yaw, self.look_ahead_course_error, self.course_error, self.distance_to_goal = self.path.calculate_error_vector(
            self.atr.state)
        if not self.params.path_param.without_anything:
            if not self.params.rangefinder: # my dirty method
                if not self.params.path_param.without_walls:
                    self.min_distance_to_walls, self.closest_point_on_walls = self.path.minimum_distance_to_walls(
                    self.atr.state[0:2], method='kdtree', num_of_closest_points=self.params.sensor_param.num_points_on_walls)

                # Print atr and obstacles info
                # print(f"[s, e] = {self.eta}")
                # print(f"target angle: {np.rad2deg(target_point_yaw)}")
                # print(f"look_ahead_course_error: {np.rad2deg(self.look_ahead_course_error)}")
                # print(f"course_error: {np.rad2deg(self.course_error)}")
                
                # Generate fake sensor data
                self.closeness, self.obstacles_obs, self.results = self.closeness_cal()
            else:
                self.intersections_, self.obstacles_obs, self.measurements, self.results, self.closeness = self.rf.closeness_cal(self.atr.state, self.path.obstacles_np)
        else:
            self.obstacles_obs = np.array([])
            self.closeness = np.zeros(self.params.sensor_param.nsectors)
            
        # observations for the agent
        next_obs = self._get_obs()
        # print(self.obstacles_obs)
        self.reward_pf, self.reward_oa, self.r_low_speed, self.reward = self.reward_calc()
        self.terminated = False
        
        self.total_reward = 0.0
        return next_obs, {}
    
    def check_point_position(self, atr_state, obs_center):
        start = atr_state[0:2]
        angle = atr_state[2]
        point = obs_center
        # Define the vector of the line based on the angle
        line_vector = [math.cos(angle), math.sin(angle)]
        
        # Compute the vector between the start of the line and the point
        point_vector = [point[0]-start[0], point[1]-start[1]]

        # Calculate the Z component of the cross product
        cross_product_z = line_vector[0]*point_vector[1] - line_vector[1]*point_vector[0]
        if cross_product_z > 0:
            return 1
        elif cross_product_z < 0:
            return 0
        else:
            return 2 # on the line
        
    def closeness_cal(self):
        """
        This method calculates the closeness of the observation. Will return closeness, all the obstacles and measurement data.
        
        ### Returns
        - `closeness` np.ndarray, (params.nsectors, ), the closeness of each sector
        - `obstacles_obs` ndarray, (n_obstacles, 3) [[distance, angle_diff, angle_diff_global], ...]
        - `result` np.ndarray, shape: (params.nsectors, ), pooling result of each sector
        """
        # Generate fake sensor data
        circles = self.path.obstacles_np.copy()
        # Point and its heading angle
        point = self.atr.state.copy()
        # Discretize theta    
        distances = []
        if_compensate = []
        angle_diffs = []
        angle_diffs_global = []
        obstacles_obs = []
        tmp_angles = []
        which_sides = []
        obs_or_walls = [] # obs: 0, walls: 1
        
        # Copy parameters
        params = self.params.sensor_param
        closest_threashold = 0.3
        # For each circle
        for circle_id, circle in enumerate(circles):
            h, k, r = circle
            # calculate the closest distance
            x_obs = h - point[0]
            y_obs = k - point[1]
            dm = np.sqrt(x_obs**2 + y_obs**2) - r # closest distance between the obs boundary and the point
            if dm > params.distance_threshold:
                continue
            # print(f"distance to the obs boundary: {dm}")
            # angle_diff = np.arctan2(k - point[1], h - point[0])
            angle_diff = np.arctan2(point[1] - k, point[0]- h)
            start_angle = angle_diff - params.narrow_angle
            # end_angle = start_angle + params.angle_inbetween
            end_angle = angle_diff + params.narrow_angle
            
            thetas = np.linspace(start_angle, end_angle, params.num_points_on_obstacles)
            
            # Calculate x and y for each theta
            xs = h + r * np.cos(thetas)
            ys = k + r * np.sin(thetas)

            # Calculate distances for each (x, y)
            distance = np.sqrt((point[0] - xs)**2 + (point[1] - ys)**2)
            distances.append(distance)
            # tmp = np.ones_like(distance) * (circle_id + 1)
            # if_compensate.append(tmp)
            
            # will check if needs the compensation for just very close obstacles or for all visible obstacles
            # print(dm)
            if dm < 0.1: # TODO: should change to a parameter
                tmp = np.ones_like(distance) * ((circle_id+1)*100)
                if_compensate.append(tmp)
            else:
                tmp = np.ones_like(distance) * (circle_id + 1)
                if_compensate.append(tmp)
            #     tmp = np.zeros_like(distance)
            #     if_compensate.append(tmp)
            which_side = self.check_point_position(self.atr.state, circle[0:2])
            which_sides.append(np.ones_like(distance) * which_side)
            
            # Calculate angle differences
            # Relative angle from x axis to obstacle
            angles = np.arctan2(ys - point[1], xs - point[0])
            angle_diffs_global.append(angles)
            
            # The relative angle from car to obstacle
            # make sure the relative angle is betwen -pi to pi
            angle_diff = angles - point[2]
            tmp_angle = angle_diff.copy() 
            angle_diff = np.remainder(angle_diff + np.pi, 2 * np.pi) - np.pi
            angle_diffs.append(angle_diff)
            tmp_angles.append(tmp_angle)
            obs_or_walls.append(np.zeros_like(distance))   
        if not self.params.path_param.without_walls:    
            # Add points on the walls to the obstacles_obs
            avg_distance_to_walls = np.min(self.min_distance_to_walls)
            wall_center = np.mean(self.closest_point_on_walls, axis=0)
            which_side = self.check_point_position(self.atr.state, wall_center)
            for distance, wallp in zip(self.min_distance_to_walls, self.closest_point_on_walls):
                angles = np.arctan2(wallp[1] - point[1], wallp[0] - point[0])
                angle_diffs_global.append(angles)
                
                # angle_diff = angles - point[2]
                angle_diff = angles - point[2]
                angle_diff = np.remainder(angle_diff + np.pi, 2 * np.pi) - np.pi
                angle_diffs.append(angle_diff)
                distances.append(distance)
                if avg_distance_to_walls < 0.2:
                    if_compensate.append(1000)
                else:
                    if_compensate.append(666)
                which_sides.append(which_side)
                obs_or_walls.append(1)
        if len(distances) != 0:    
            distances = np.hstack(distances)
            distances = distances.flatten()
            if_compensate = np.hstack(if_compensate)
            if_compensate = if_compensate.flatten() 
            angle_diffs = np.hstack(angle_diffs)
            angle_diffs = angle_diffs.flatten()
            angle_diffs_global = np.hstack(angle_diffs_global)
            angle_diffs_global = angle_diffs_global.flatten()
            which_sides = np.hstack(which_sides)
            which_sides = which_sides.flatten()
            obs_or_walls = np.hstack(obs_or_walls)
            obs_or_walls = obs_or_walls.flatten()
        
        how_many_obs = 0
        for idx, distance in enumerate(distances):
            # check if the obs point is within the sensor range(distance and angle)
            if if_compensate[idx] % 100 == 0: # this will include all points on the circle, so that 
                obstacles_obs.append([distance, angle_diffs[idx], angle_diffs_global[idx], if_compensate[idx], which_sides[idx], obs_or_walls[idx]])
            elif distance < params.distance_threshold and np.abs(angle_diffs[idx]) < np.deg2rad(120): # TODO: params.angle_threshold 
                obstacles_obs.append([distance, angle_diffs[idx], angle_diffs_global[idx], if_compensate[idx], which_sides[idx], obs_or_walls[idx]])
        
                
        # obstacles_obs: distance|angle differs|angle differs global
        # Generate Sectors
        num_of_sectors = params.nsectors + 1
        
        # Create an array to store the extracted data based on the sectors
        result = np.ones(num_of_sectors) * 500 # give a initial very large number
        counts = np.zeros(num_of_sectors)
        
        start_sector_indices = []
        end_sector_indices = []
        
        if len(obstacles_obs) != 0:
            obstacles_obs = np.array(obstacles_obs).reshape(-1, 6)
            how_many_obs = np.count_nonzero(obstacles_obs[:, 5] == 0)
            # print(f"obstacles_obs:\n {obstacles_obs}")
            # print(how_many_obs)
            mask = (obstacles_obs[:, 1] < 0) & (obstacles_obs[:, 3] == 1000) & (obstacles_obs[:, 4] == 1) & (obstacles_obs[:, 5] == 1)
            obstacles_obs[mask, 1] = np.deg2rad(360) - np.abs(obstacles_obs[mask, 1])
            mask = (obstacles_obs[:, 1] > 0) & (obstacles_obs[:, 3] == 1000) & (obstacles_obs[:, 4] == 0) & (obstacles_obs[:, 5] == 1)
            obstacles_obs[mask, 1] = -(np.deg2rad(360) - np.abs(obstacles_obs[mask, 1]))
            sort_indices = np.argsort(obstacles_obs[how_many_obs:, 1])
            obstacles_obs[how_many_obs:] = obstacles_obs[how_many_obs:][sort_indices]
            # print(f"obstacles_obs:\n {obstacles_obs}")
            
            compensate_indices = obstacles_obs[:, 3].T # (n_obstacles,) e.g (0, 0, 0, 1, 1, 1, 2, 2, 2)
            ### Get start and end indices for each data that needs to be compensated
            starts, ends = self.extract_startend_indices(compensate_indices) # [3,6], [5,8]
            # Pooling data in each sector
            # Iterate over each angle in the measurement data and find the corresponding sector
            for idx, data in enumerate(obstacles_obs):
                
                # if data[4] == 1 and data[1] < 0:
                #     tmpdata = np.deg2rad(360) - abs(data[1])
                # elif data[4] == 0 and data[1] > 0:
                #     tmpdata = - (np.deg2rad(360) - abs(data[1]))
                # else:
                #     tmpdata = data[1]
                # Find the index of the sector in which the angle falls
                # sector_index = np.searchsorted(params.sectors, tmpdata) # data[1]
                # print(np.rad2deg(data[1]))
                # print(f"sector_index: {sector_index}")
                
                # if data[4] == 1 and data[1] < 0:
                #     tmpdata = np.deg2rad(360) - abs(data[1])
                #     sector_index = np.searchsorted(params.sectors, tmpdata)
                # elif data[4] == 0 and data[1] > 0:
                #     tmpdata = - (np.deg2rad(360) - abs(data[1]))
                #     sector_index = np.searchsorted(params.sectors, tmpdata)
                # else:
                #     sector_index = np.searchsorted(params.sectors, data[1])
                if (idx in starts):
                    if data[4] == 1 and data[1] < 0:
                        tmpdata = np.deg2rad(360) - abs(data[1])
                        sector_index = np.searchsorted(params.sectors, tmpdata) # data[1]
                    else:
                        sector_index = np.searchsorted(params.sectors, data[1])
                    if sector_index >= num_of_sectors:
                        sector_index = num_of_sectors - 1
                    end_sector_indices.append(sector_index)
                
                elif (idx in ends):
                    if data[4] == 0 and data[1] > 0:
                        tmpdata = - (np.deg2rad(360) - abs(data[1]))
                        sector_index = np.searchsorted(params.sectors, tmpdata) # data[1]
                    else:
                        sector_index = np.searchsorted(params.sectors, data[1])
                    if sector_index >= num_of_sectors:
                        sector_index = num_of_sectors - 1 
                    start_sector_indices.append(sector_index)
                else:
                    if abs(data[1]) > self.params.sensor_param.angle_threshold:
                        continue
                    sector_index = np.searchsorted(params.sectors, data[1])    
                    if sector_index >= num_of_sectors:
                        sector_index = num_of_sectors - 1
                # # Find the index of the sector in which the angle falls
                # sector_index = np.searchsorted(params.sectors, data[1])
                # if sector_index >= num_of_sectors:
                #     sector_index = num_of_sectors - 1
                # if idx in starts:
                #     end_sector_indices.append(sector_index)
                # if idx in ends:
                #     start_sector_indices.append(sector_index)
                
                # Add the distance value to the corresponding sector in the result array
                ## TODO:
                # Using average here is not very suitable. Consider the case two obstacles are in series, thus will get one very small and one very large measurement.
                # If average the these two then the result will be larger than the dangerous distance. Should consider taking more from smaller one.
                # weighted average?
                if data[0] <= result[sector_index]:
                    result[sector_index] = data[0]
                # result[sector_index] += data[0]
                counts[sector_index] = 1
            start_sector_indices = np.array(start_sector_indices)
            end_sector_indices = np.array(end_sector_indices)
            
            mask = start_sector_indices > end_sector_indices
            start_sector_indices[mask], end_sector_indices[mask] = end_sector_indices[mask], start_sector_indices[mask]
    
            
            # averaging pooling
            result = np.divide(result, counts, out=np.zeros_like(result), where=counts!=0)
            result = np.where(result == 0, 1.0, result) # the second parameter "1" controls the maximum value of result. Need to check if should be set as 1 or the threshold
            # print(f"result: {result}")
            # print(f"start_sector_indices: {start_sector_indices}")
            # print(f"end_sector_indices: {end_sector_indices}")
            result = self.smooth_data(result, start_sector_indices, end_sector_indices)
            # print(f"after smoth: {result}")
            result = result[1:]
            closeness = 1-1*result
        else:
            result = np.ones(num_of_sectors-1) * 1.0
            closeness = np.zeros(params.nsectors)
        return closeness, np.array(obstacles_obs), result
    
    def extract_startend_indices(self, data):
        # Create an array of differences
        diffs = np.diff(data)
        
        # Initialize lists to hold start and end indices
        starts = []
        ends = []

        # Check if the first element is non-zero
        if data[0] != 0:
            starts.append(0)

        # Iterate through differences
        for i, diff in enumerate(diffs):
            # If the difference is not zero, a new data flow starts or ends
            if diff != 0:
                # If data[i] is not zero, then it's the end of a data flow
                if data[i] != 0:
                    ends.append(i)
                    # Check if the length of the data flow is greater than 1
                    if i - starts[-1] == 0:
                        starts.pop()
                        ends.pop()
                # If data[i + 1] is not zero, then it's the start of a data flow
                if data[i + 1] != 0:
                    starts.append(i + 1)

        # Handle the end of the last data flow
        if data[-1] != 0:
            ends.append(len(data) - 1)
            # Check if the length of the data flow is greater than 1
            if ends[-1] - starts[-1] == 0:
                starts.pop()
                ends.pop()
        return starts, ends

    def smooth_data(self, data, start_indices, end_indices):
        """
        Smooth the result by sector indices.
        """
        # Ensure the start_indices and end_indices lists have the same length
        assert len(start_indices) == len(end_indices), "The start_indices and end_indices lists must have the same length."

        # For each pair of start and end indices
        for start_index, end_index in zip(start_indices, end_indices):
            # Get the interval between the specified indices
            interval = data[start_index:end_index+1]
            
            # Calculate the average of the non-'1' values in the interval
            non_ones = interval[interval != 1]
            average = np.mean(non_ones)
            
            # # For each value in the interval
            # for i, value in enumerate(interval):
            #     # If the value is larger than the average
            #     if value > average:
            #         # Replace the value with the average
            #         interval[i] = average
            #     # If the value is smaller than the average
            #     elif value < average:
            #         # Keep the smaller value
            #         interval[i] = value
            # # Replace all '1' values in the interval with the average
            # # interval[interval == 1] = average
            interval[interval > average] = average
            
            # Replace the original data with the smoothed interval
            data[start_index:end_index+1] = interval
        return data