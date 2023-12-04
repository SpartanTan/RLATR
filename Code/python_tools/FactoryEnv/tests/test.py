import argparse
import gymnasium as gym
import numpy as np
from factory_env.envs.parameters import env_param
import matplotlib.pyplot as plt
import time
import random

import sys
import termios
import tty
import threading
import matplotlib

import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Categorical, Normal

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
    
plt.ion()
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--which_test', type=int, default=0, help='Which test to perform? 0: manual, 1: benchmark, 2: test agent')
    parser.add_argument('--which_option', type=int, default=0, help='Which test to perform? 0: run, 1: save, 2: load')
    
    parser.add_argument('--agent_dir', type=str, default="agents/", help='Which agent directory to load?')
    parser.add_argument('--agent_file', type=str, default=None, help='Which agent file to load?')
    
    parser.add_argument('--No', type=int, default=8, help='Description for No')
    parser.add_argument('--Nw', type=int, default=8, help='Description for Nw')
    parser.add_argument('--Lp', type=int, default=6, help='Description for Lp')
    parser.add_argument('--mu_r', type=float, default=0.25, help='Description for mu_r')
    parser.add_argument('--sigma_d', type=float, default=0.5, help='Description for sigma_d')
    parser.add_argument('--shift_distance', type=float, default=1, help='Description for shift_distance')
    parser.add_argument('--extend_length', type=float, default=2.0, help='Description for extend_length')
    parser.add_argument('--look_ahead_distance', type=float, default=0.5, help='Description for look_ahead_distance')
    parser.add_argument('--target_finishing_time', type=int, default=30, help='Description for target_finishing_time')
    parser.add_argument('--without_walls', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle')
    parser.add_argument('--allowed_jump_index', type=int, default=5, help='number of points the robot can jump on the path')
    parser.add_argument('--how_many_points_forward', type=int, default=2, help='Description for how_many_points_forward')
    parser.add_argument('--max_ep_steps', type=int, default=500, help='Maximum steps allowed in one episode')
    parser.add_argument('--obs_mean_vel', type=float, default=0.1, help='mean velocity of the obstacles')
    parser.add_argument('--obs_std_vel', type=float, default=0.1, help='standard deviation of the velocity of the obstacles')
    parser.add_argument('--flip_chance', type=float, default=0.3, help='The chace to flip the direction of obstacles')
    parser.add_argument('--dynamic_obstacles_r', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle the raidus of the obstacles')
    parser.add_argument('--dynamic_obstacles', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle the movement of the obstacles')
    parser.add_argument('--static_map', type=int, default=0, help='0: run, 1: save, 2: load')
    parser.add_argument('--consider_width', type=int, default=0, help='whether to consider width in calculations (0 or 1)')

    parser.add_argument('--dt', type=float, default=0.1, help='time step')
    parser.add_argument('--wheel_radius', type=float, default=0.125, help='Description for wheel_radius')
    parser.add_argument('--track_width', type=float, default=0.48, help='Description for track_width')
    parser.add_argument('--atr_linear_vel_max', type=float, default=0.2, help='Description for atr_linear_vel_max')
    

    parser.add_argument('--full_plot', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle')
    parser.add_argument('--rangefinder', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle')
    
    parser.add_argument('--distance_threshold', type=float, default=0.8, help='Description for distance_threshold')
    parser.add_argument('--sensor_angle', type=int, default=240, help='Description for sensor_angle')
    parser.add_argument('--nsectors', type=int, default=20, help='Description for nsectors')
    parser.add_argument('--num_points_on_walls', type=int, default=5, help='Description for num_points_on_walls')
    parser.add_argument('--num_points_on_obstacles', type=int, default=5, help='Description for num_points_on_obstacles')
    parser.add_argument('--narrow_angle', type=float, default=60.0, help='Description for narrow_angle')
    parser.add_argument('--angle_inbetween', type=float, default=120.0, help='Description for angle_inbetween')
    parser.add_argument('--resolution', type=float, default=6.0, help='Description for resolution')
    parser.add_argument('--Sr', type=float, default=1.0, help='Description for Sr')

    parser.add_argument('--alpha_lambda', type=float, default=1.0, help='trade-off paramter')
    parser.add_argument('--beta_lambda', type=float, default=2.0, help='trade-off paramter')
    parser.add_argument('--gamma_theta', type=float, default=4.0, help='obstacle avoidance reward parameter')
    parser.add_argument('--gamma_x', type=float, default=0.1, help='obstacle avoidance reward parameter') 
    parser.add_argument('--epsilon_x', type=float, default=0.1, help='deadzone distance of obstacle avoidance reward') 
    parser.add_argument('--gamma_e', type=float, default=0.5, help='path following reward parameter')
    parser.add_argument('--alpha_r', type=float, default=0.1, help='existance reward parameter')
    parser.add_argument('--r_collision', type=float, default=-20000, help='Description for r_collision')
    parser.add_argument('--r_arrived', type=float, default=20000, help='Description for r_arrived')
    parser.add_argument('--r_terminated', type=float, default=-20000, help='Description for r_terminated')
    parser.add_argument('--max_total_reward', type=float, default=10000, help='Description for max_total_reward')
    parser.add_argument('--lagging_frac', type=float, default=3, help='Description for lagging_frac')
    parser.add_argument('--r_oa_frac', type=float, default=5, help='Description for r_oa_frac')
    parser.add_argument('--angle_resolution', type=float, default=0.1, help='step size for the angle in oa')
    parser.add_argument('--distance_resolution', type=float, default=0.05, help='step size for the distance in oa')
    parser.add_argument('--oa_result_resolution', type=float, default=0.3, help='step size for the result in oa')
    
    parser.add_argument('--which_env', type=int, default=0, help='Select which environment to run')
    args = parser.parse_args()
    return args
       
class NewTeleopControl:
    def __init__(self, ratio):
        self.ratio = ratio
        self.key = 'e'
        self.std_speed = np.array([1.0, 1.0])
        self.running = True

    def get_key(self):
        # fd = sys.stdin.fileno()
        # old_settings = termios.tcgetattr(fd)
        # try:
        #     tty.setraw(sys.stdin.fileno())
        #     ch = sys.stdin.read(1)
        #     if ch == 'q':  # Quit condition
        #         self.running = False
        #     if ch == '\x1b':
        #         ch += sys.stdin.read(1)
        #         ch += sys.stdin.read(1)
        # finally:
        #     termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        ch = sys.stdin.read(1)
        return ch
    
    def get_action(self):
        spd = 1.0 #0.192
        if self.is_pressed('w'): # forward
            action = self.std_speed * self.ratio
        elif self.is_pressed('s'): # backwards
            action = -self.std_speed * self.ratio
        elif self.is_pressed('a'): # rotate left
            action = np.array([spd, -spd]) * self.ratio
        elif self.is_pressed('d'): # rotate right
            action = np.array([-spd, spd]) * self.ratio
        elif self.is_pressed('j'): # forward left
            action = np.array([spd, spd/2]) * self.ratio
        elif self.is_pressed('k'): # forward right
            action = np.array([spd/2, spd]) * self.ratio
        elif self.is_pressed('n'): # backward left
            action = np.array([-spd, -spd/2]) * self.ratio
        elif self.is_pressed('m'): # backward right
            action = np.array([-spd/2, -spd]) * self.ratio
        elif self.is_pressed('e'): # stop
            action = np.array([0.0, 0.0]) * self.ratio
        return action
    
    def is_pressed(self, ch):
        if ch == self.key:
            return True

    def run_keyboard(self):
        while self.running:
            self.key = self.get_key()    

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer
    
class Agent(nn.Module):
    def __init__(self):
        super().__init__()
        self.critic = nn.Sequential(
               layer_init(nn.Linear(np.array(env.observation_space.shape).prod(), 64)), # input feature: 26, output feature: 64
               nn.Tanh(),
               layer_init(nn.Linear(64, 64)),
               nn.Tanh(),
               layer_init(nn.Linear(64, 1), std=1.0)
          )
        self.actor_means = nn.Sequential(
            layer_init(nn.Linear(np.array(env.observation_space.shape).prod(), 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, np.prod(env.action_space.shape)), std=0.01) # 0.01 make the probability of taking each action be similar
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, np.prod(env.action_space.shape))) # 2-dim vector, each dim is the logstd for each action
        
    def get_value(self, x):
        """
        ### Returns
        `value` [n_envs, 1]
        """
        return self.critic(x)

    def get_action_and_value(self, x, action=None):
        """
        This method gets action and V(s) by feeding observation array into the critic and actor network.
        Since it's a continuous environment. First get action_mean from actor NN, then get normal distribution based on the mean ad standard variance.
        Sample from the distribution to get action output. 
        ## TODO: Should give it a action constrain right?
        
        If action is given, it will still go through the procedure above, then find the log_prob of the given action of the distribution given by NN from current state.
        
        ### Parameters:
        `x` observation array. [N, 26]. could be [n_envs, 26] or [minibatch_size, 26] \ 
        `action` If given this parameter, calculate the log_prob of this action in the normal distribution from NN(obs)
        
        ### Returns
        `action` [N, n_actions] \ 
        `logprob` [N, 1]  \ 
        `entropy` [N, 1]  \  
        `value` [N, 1] 
        """
        # if torch.any(torch.isnan(x)):
        #     print(x)
        #     raise ValueError("x is nan")
        action_mean = self.actor_means(x)
        # print("Shape of next_obs: ", next_obs.shape)
        # print("Shape of action_mean: ", action_mean.shape)
        # print("Shape of actor_logstd: ", self.actor_logstd.shape)
        # raise
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = Normal(action_mean, action_std)
        
        if action is None:
            action = probs.sample()

        return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.critic(x)   
    
"""
 
   ____  _____    _    _       _____ _   ___     __
  |  _ \| ____|  / \  | |     | ____| \ | \ \   / /
  | |_) |  _|   / _ \ | |     |  _| |  \| |\ \ / / 
  |  _ <| |___ / ___ \| |___  | |___| |\  | \ V /  
  |_| \_\_____/_/   \_\_____| |_____|_| \_|  \_/   
                                                   
 
"""
# package_dir = pkg_resources.get_distribution('FactoryEnv').location
# map_dir = package_dir + '/factory_env/map/config/nona_description.json'
# path_dir = package_dir + '/factory_env/map/config/nodes_tuve.json'
# path_name = "OnePath"
# atr_init_position = np.array([6.0, 6.0])
# env = gym.make('real-factory-v0', map_dir=map_dir, path_dir=path_dir, path_name=path_name, atr_init_position=atr_init_position, render_mode="human")

# action = env.action_space.sample()
# print(action)
# observation, info = env.reset()
# print(observation)
# print(env.observation_space.sample())
# env.render()

"""
 
   _____ ____      _    ___ _   _   _____ _   ___     __
  |_   _|  _ \    / \  |_ _| \ | | | ____| \ | \ \   / /
    | | | |_) |  / _ \  | ||  \| | |  _| |  \| |\ \ / / 
    | | |  _ <  / ___ \ | || |\  | | |___| |\  | \ V /  
    |_| |_| \_\/_/   \_\___|_| \_| |_____|_| \_|  \_/   
                                                        
 
"""
if __name__ == "__main__":
    args = parse_args()
    if args.which_option == 0:
        options = {'init_type': 'run', 'file_name': 'test.pkl'}
    elif args.which_option == 1:   
        options = {'init_type': 'save', 'file_name': 'test.pkl'}
    elif args.which_option == 2:
        options = {'init_type': 'load', 'file_name': 'test.pkl'}
        
    # load_network_name = "good_agents/training-factory-v0_PPO_1_1690299629.pth"   
    # load_network_name = "agents/training-factory-v0_PPO_1_1699453078.pth"
    params:env_param = env_param()
    # load_arguments_into_params(params, args)
    # check_param(params, args)
    params.load_arguments(args)
    params.check_param()

    torch.manual_seed(1)
    torch.backends.cudnn.deterministic=True
    map_dir = "/home/zhicun/code/atr_rl/Code/python_tools/FactoryEnv/factory_env/map/config/nona_description.json"
    path_dir = "/home/zhicun/code/atr_rl/Code/python_tools/FactoryEnv/factory_env/map/config/nodes_tuve.json"
    path_name = "OnePath" # testPath OnePath
    if args.which_env == 0:
        env = gym.make('training-factory-v0', params=params,)
    elif args.which_env == 1:
        env = gym.make('real-factory-v0', params=params, map_dir=map_dir, path_dir=path_dir, path_name=path_name, render_mode="human")

    # action = env.action_space.sample()
    # print(action)
    next_obs, info = env.reset(options=options)
    env.unwrapped.path.print_shape()

    start_time = time.time()
    elapsed_time = 0.0
    fake_time = 0.0
    compute_time = 0.0

    # action = np.array([1.6, 1.6])

    teleop = NewTeleopControl(1)
    thread = threading.Thread(target=teleop.run_keyboard)
    thread.start()
    steps = 0

    if args.which_test == 0:
        while True:
            steps += 1
            loop_time = time.time()
            # plt.cla()
            # input("Press 'space' to continue...")
            action = teleop.get_action()
            # action = env.action_space.sample()
            observation, reward, done, truncated, info = env.step(action)
            if done:
                print(steps)
                env.render()
                plt.show(block=True)
                break
            
            env.render()
            # if env.ghost_index == len(env.path.ghost_trajectory) - 1:
            #     print(steps)
            #     print(elapsed_time)
            #     break
            pause_time = 0.1 - compute_time
            compute_time = time.time() - loop_time
            # print(compute_time)
            if pause_time <= 0:
                pause_time = 1e-3
            fake_time += 0.1
            # print(env.atr.state)
            # plt.title(f"elapsed time: {round(elapsed_time,2)} s, comp time: {round(compute_time, 4)} s")
            #   plt.show(block=False)
            plt.pause(pause_time)
            # time.sleep(pause_time)
            elapsed_time = time.time() - start_time
            # print(f"elapsed time: {round(elapsed_time,2)} s, comp time: {round(fake_time, 4)} s")
        next_obs, info = env.reset(options=options)
    
    elif args.which_test == 1:
        success_rate = 0
        N_episodes = 100
        N_success = 0
        finishi_rate = []
        steps_collect = []
        cross_track_error = []
        avg_finish_rate = 0
        agent_dir = args.agent_dir 
        load_network_name = agent_dir + '/' + args.agent_file
        '''
                             _     _                 _                   _   
           __ _ __ _ ___ _ _| |_  | |__  ___ _ _  __| |_  _ __  __ _ _ _| |__
          / _` / _` / -_) ' \  _| | '_ \/ -_) ' \/ _| ' \| '  \/ _` | '_| / /
          \__,_\__, \___|_||_\__| |_.__/\___|_||_\__|_||_|_|_|_\__,_|_| |_\_\
               |___/                                                         
        '''
        print(f"Benchmarking: "+ '\'' + load_network_name + '\'')
        agent:Agent = Agent()
        agent.load_state_dict(torch.load(load_network_name))
        next_obs = torch.Tensor(next_obs)
        for test in range(N_episodes):
            while True:
                steps += 1
                loop_time = time.time()
                # action = env.action_space.sample()
                with torch.no_grad():
                    action, logprob, _, value = agent.get_action_and_value(next_obs.unsqueeze(0))
                
                next_obs, reward, done, truncated, info = env.step(action.numpy()[0])
                cross_track_error.append(next_obs[5])
                next_obs = torch.Tensor(next_obs)
                if done:
                    if info['is_arrived']:
                        N_success += 1
                    if info['finish_rate']:
                        finishi_rate.append(info['finish_rate'])
                    if info['steps']:
                        steps_collect.append(info['steps'])
                    success_rate = N_success / (test + 1)
                    print(steps)
                    next_obs, info = env.reset()
                    next_obs = torch.Tensor(next_obs)
                    steps = 0
                    break
                
                # env.render()
                # plt.pause(0.1)

        avg_finish_rate = np.mean(finishi_rate)
        avg_steps = np.mean(steps_collect)
        avg_cross_track_error = np.mean(cross_track_error)
        print(f"Avg finish rate: {avg_finish_rate}")
        print(f"Success rate: {success_rate}")
        print(f"Avg steps: {avg_steps}")
        print(f"Avg cross track error: {avg_cross_track_error}")
    
    elif args.which_test == 2:
        agent_dir = args.agent_dir 
        load_network_name = agent_dir + '/' + args.agent_file
        cross_track_error = []
        robot_pos = [] # 2d coordinates will be appended
        wheel_speed = []
        """
 
                                  _     _            _   
            __ _  __ _  ___ _ __ | |_  | |_ ___  ___| |_ 
           / _` |/ _` |/ _ \ '_ \| __| | __/ _ \/ __| __|
          | (_| | (_| |  __/ | | | |_  | ||  __/\__ \ |_ 
           \__,_|\__, |\___|_| |_|\__|  \__\___||___/\__|
                 |___/                                   
 
        """
        print(f"Testing: "+ '\'' + load_network_name + '\'')
        agent:Agent = Agent()
        agent.load_state_dict(torch.load(load_network_name))
        next_obs = torch.Tensor(next_obs)
        while True:
            steps += 1
            loop_time = time.time()
            # action = env.action_space.sample()
            with torch.no_grad():
                action, logprob, _, value = agent.get_action_and_value(next_obs.unsqueeze(0))
                rl = action.numpy()[0]
                # clip the action
                # rl = np.clip(rl, -1.6, 1.6)

                wheel_speed.append(rl)
            next_obs, reward, done, truncated, info = env.step(rl)
            cross_track_error = np.append(cross_track_error, next_obs[5])
            robot_pos.append([env.unwrapped.atr.state[0], env.unwrapped.atr.state[1]])
            next_obs = torch.Tensor(next_obs)
            if done:
                print(steps)
                env.render()

                fig, ax = plt.subplots()
                ax.plot(cross_track_error)
                ax.set(xlabel='time (s)', ylabel='cross track error (m)', title='Cross Track Error')
                ax.grid()

                fig1, ax1 = plt.subplots()
                ax1.plot(wheel_speed)
                ax1.set(xlabel='time (s)', ylabel='wheel speed (m/s)', title='Wheel Speed')
                ax1.grid()
                
                fig, ax = plt.subplots()
                robot_pos = np.array(robot_pos)
                ax.plot(robot_pos[:,0], robot_pos[:,1], 'r--')
                env.unwrapped.path.render(ax)
                # env.unwrapped.polygon_map.render(ax)
                # ax.plot(env.unwrapped.path.even_trajectory[:,0], env.unwrapped.path.even_trajectory[:,1])
                # for idx, obs in enumerate(env.unwrapped.path.obstacles):
                #     p_obst, r_obst = env.unwrapped.path.obstacles_np[idx, :2], env.unwrapped.path.obstacles_np[idx, 2]
                #     circle = plt.Circle((p_obst[0], p_obst[1]), r_obst, color='r', alpha=0.5)
                #     ax.add_patch(circle)
                #     ax.text(p_obst[0], p_obst[1], idx+1, fontsize=12)   
                ax.set(xlabel='x (m)', ylabel='y (m)', title='Robot Position')
                ax.axis('equal')
                ax.grid()

                plt.show(block=True)
                break
            
            env.render()
            # if env.ghost_index == len(env.path.ghost_trajectory) - 1:
            #     print(steps)
            #     print(elapsed_time)
            #     break
            pause_time = 0.1 - compute_time
            compute_time = time.time() - loop_time
            # print(compute_time)
            if pause_time <= 0:
                pause_time = 1e-3
            fake_time += 0.1
            # print(env.atr.state)
            # plt.title(f"elapsed time: {round(elapsed_time,2)} s, comp time: {round(compute_time, 4)} s")
            #   plt.show(block=False)
            plt.pause(pause_time)
            # time.sleep(pause_time)
            elapsed_time = time.time() - start_time
            # print(f"elapsed time: {round(elapsed_time,2)} s, comp time: {round(fake_time, 4)} s")
    

"""

__     _______ ____ _____ _   ___     __
\ \   / / ____/ ___| ____| \ | \ \   / /
\ \ / /|  _|| |   |  _| |  \| |\ \ / / 
    \ V / | |__| |___| |___| |\  | \ V /  
    \_/  |_____\____|_____|_| \_|  \_/   
                                        

"""


def make_env(gym_id='training-factory-v0'):
    def thunk():
        # env = CartPoleEnv()
        env = gym.make(gym_id, params=params,
                    use_saved_map=False, render_mode="human")
        # env = gym.wrappers.RecordEpisodeStatistics(env)
        # if capture_video:
        #     if idx == 0:
        #         env = gym.wrappers.RecordVideo(env, f"videos/{run_name}")
        # # env.seed(args.seed)
        # env.action_space.seed(seed)
        # env.observation_space.seed(seed)
        return env
    return thunk


# envs = gym.vector.SyncVectorEnv([make_env() for i in range(4)])
# print("envs.single_observation_space.shape", envs.single_observation_space.shape)
# print("envs.single_action_space.shape", envs.single_action_space.shape)
# next_obs, info = envs.reset()
# print(next_obs)
# print(next_obs.shape)
