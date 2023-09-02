import argparse
import gymnasium as gym
import numpy as np
from factory_env.envs.parameters import env_param
import matplotlib.pyplot as plt
import time

import sys
import termios
import tty
import threading
import matplotlib

import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Categorical, Normal

plt.ion()
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--which-test', type=int, default=0, help='Which test to perform? 0: manual, 1: benchmark, 2: test agent')
    parser.add_argument('--which-option', type=int, default=0, help='Which test to perform? 0: run, 1: save, 2: load')
    args = parser.parse_args()
    return args
       
class NewTeleopControl:
    def __init__(self, ratio):
        self.ratio = ratio
        self.key = 'e'
        self.std_speed = np.array([0.8, 0.8])
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
        if self.is_pressed('w'):
            action = self.std_speed * self.ratio
        elif self.is_pressed('s'):
            action = -self.std_speed * self.ratio
        elif self.is_pressed('a'):
            action = np.array([0.8, -0.8]) * self.ratio
        elif self.is_pressed('d'):
            action = np.array([-0.8, 0.8]) * self.ratio
        elif self.is_pressed('e'):
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
        
    load_network_name = "good_agents/training-factory-v0_PPO_1_1690299629.pth"   
    # load_network_name = "agents/training-factory-v0_PPO_1_1690383283.pth"
    
    params:env_param = env_param()
    params.atr_param.atr_linear_vel_max = 0.2
    params.path_param.target_finishing_time = 30
    check_param(params)
    
                            
    env = gym.make('training-factory-v0', params=params,)
    # action = env.action_space.sample()
    # print(action)
    next_obs, info = env.reset(options=options)
    env.path.print_shape()

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
        """
        
        _                     _                          _     
        | |__   ___ _ __   ___| |__  _ __ ___   __ _ _ __| | __ 
        | '_ \ / _ \ '_ \ / __| '_ \| '_ ` _ \ / _` | '__| |/ / 
        | |_) |  __/ | | | (__| | | | | | | | | (_| | |  |   <  
        |_.__/ \___|_| |_|\___|_| |_|_| |_| |_|\__,_|_|  |_|\_\ 
                                                                
        
        """
        start_time = time.time()
        action = np.array([0.0, 0.0])
        # for i in range(300):
        while True:
            steps += 1
            observation, reward, done, truncated, info = env.step(action)
            # env.render()
            # plt.pause(0.01)
            if env.ghost_index == len(env.path.ghost_trajectory) - 1:
                print(steps)
                elapsed_time = time.time() - start_time
                print(elapsed_time)
                target_time = env.path.trajectory_length_at_each_point[-1]/params.atr_param.atr_linear_vel_max
                print(target_time)
                break
    
    elif args.which_test == 2:
        """
 
                                  _     _            _   
            __ _  __ _  ___ _ __ | |_  | |_ ___  ___| |_ 
           / _` |/ _` |/ _ \ '_ \| __| | __/ _ \/ __| __|
          | (_| | (_| |  __/ | | | |_  | ||  __/\__ \ |_ 
           \__,_|\__, |\___|_| |_|\__|  \__\___||___/\__|
                 |___/                                   
 
        """
        print("test agent")
        agent:Agent = Agent()
        agent.load_state_dict(torch.load(load_network_name))
        next_obs = torch.Tensor(next_obs)
        while True:
            steps += 1
            loop_time = time.time()
            # action = env.action_space.sample()
            with torch.no_grad():
                action, logprob, _, value = agent.get_action_and_value(next_obs.unsqueeze(0))
            
            next_obs, reward, done, truncated, info = env.step(action.numpy()[0])
            next_obs = torch.Tensor(next_obs)
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
