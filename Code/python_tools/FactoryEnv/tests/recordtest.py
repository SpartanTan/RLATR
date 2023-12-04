import argparse
import os
from distutils.util import strtobool
import time
from torch.utils.tensorboard import SummaryWriter
import random
import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Categorical, Normal

import gymnasium as gym

from factory_env.envs.parameters import env_param, path_param, reward_param, ATR_param, sensor_param
import threading

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp-name', type=str, default=os.path.basename(__file__).rstrip(".py"), help='the name of this experiment')
    parser.add_argument('--gym-id', type=str, default="training-factory-v0", help='the id of the gym environment')
    parser.add_argument('--learning-rate', type=float, default=2e-4, help='the learning rate of the optimizer')
    parser.add_argument('--seed', type=int, default=1, help='seed of the experiment')
    parser.add_argument('--total-timesteps', type=int, default=200000, help='total timesteps of the experiments')
    parser.add_argument('--torch-deterministic', type=lambda x:bool(strtobool(x)), default=True, nargs='?', const=True, help='if toggled, `torch.backends.cudnn.deterministic=False`')
    parser.add_argument('--cuda', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='if toggled, cuda will not be enabled by default')
    parser.add_argument('--track', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='if toggled, the experiment will be tracked with Weights and Biases')
    parser.add_argument("--wandb-project-name", type=str, default="ATR-RL", help="the wandb's project name")
    parser.add_argument("--wandb-entity", type=str, default=None, help="the entity (team) of wandb's project")
    parser.add_argument("--capture-video", type=lambda x: bool(strtobool(x)), default=False, nargs="?", const=True, help="weather to capture videos of the agent performances (check out `videos` folder)")
    parser.add_argument('--num-envs', type=int, default=4, help="the number of parallel game environments to run")
    parser.add_argument('--num-steps', type=int, default=1024, help="the number of steps per game environment to run") # Rollouts Data: 4*300=1200
    parser.add_argument('--anneal-lr', type=lambda x: bool(strtobool(x)), default=True, nargs='?', const=True, help="Toggle learning rate annealing for policy and value networks")
    parser.add_argument('--gae', type=lambda x: bool(strtobool(x)), default=True, nargs='?', const=True, help="Use GAE for advantage estimation")
    parser.add_argument('--gamma', type=float, default=0.999, help="discount factor gamma")
    parser.add_argument('--gae-lambda', type=float, default=0.95, help="lambda for GAE")
    parser.add_argument('--num-minibatches', type=int, default=32, help="the number of mini-batches per update")
    parser.add_argument('--update-epochs', type=int, default=10, help="the K epochs to update the policy")
    parser.add_argument('--norm-adv', type=lambda x: bool(strtobool(x)), default=True, nargs='?', const=True, help="Toggle advantage normalization")
    parser.add_argument('--clip-coef', type=float, default=0.2, help="the surrogate clipping coefficient")
    parser.add_argument('--clip-vloss', type=lambda x: bool(strtobool(x)), default=True, nargs='?', const=True, help="Toggle clipping of the value function loss")
    parser.add_argument('--ent-coef', type=float, default=0.0, help="coefficient of the entropy loss") # c2 in the paper
    parser.add_argument('--vf-coef', type=float, default=0.5, help="coefficient of the value function loss") # c1 in the paper
    parser.add_argument('--max-grad-norm', type=float, default=0.5, help="the maximum norm for the gradient clipping")
    parser.add_argument('--target-kl', type=float, default=None, help="the target KL divergence threashold")
    
    # Environment parameters
    parser.add_argument('--which_test', type=int, default=0, help='Which test to perform? 0: manual, 1: benchmark, 2: test agent')
    parser.add_argument('--which_option', type=int, default=0, help='Which test to perform? 0: run, 1: save, 2: load')
    parser.add_argument('--No', type=int, default=8, help='Description for No')
    parser.add_argument('--Nw', type=int, default=8, help='Description for Nw')
    parser.add_argument('--Lp', type=int, default=6, help='Description for Lp')
    parser.add_argument('--mu_r', type=float, default=0.25, help='Description for mu_r')
    parser.add_argument('--sigma_d', type=float, default=0.5, help='Description for sigma_d')
    parser.add_argument('--shift_distance', type=int, default=1, help='Description for shift_distance')
    parser.add_argument('--extend_length', type=float, default=2.0, help='Description for extend_length')
    parser.add_argument('--look_ahead_distance', type=float, default=0.5, help='Description for look_ahead_distance')
    parser.add_argument('--target_finishing_time', type=int, default=30, help='Description for target_finishing_time')
    parser.add_argument('--without_walls', type=lambda x:bool(strtobool(x)), default=False, nargs='?', const=True, help='toggle')
    parser.add_argument('--how_many_points_forward', type=int, default=2, help='Description for how_many_points_forward')
    parser.add_argument('--max_ep_steps', type=int, default=500, help='Description for max_ep_steps')
    
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

    args = parser.parse_args()
    args.batch_size = int(args.num_envs * args.num_steps) # 4 * 300 = 1200
    args.minibatch_size = int(args.batch_size // args.num_minibatches) # 1200 // 6 = 200

    return args


if __name__ == "__main__":
    args = parse_args()
    params = env_param()
    params.load_arguments(args)
    params.check_param()

    env = gym.make("training-factory-v0", params=params, render_mode="rgb_array")
    # env = gym.make("CartPole-v1", render_mode="rgb_array")
    env = gym.wrappers.RecordEpisodeStatistics(env)
    env = gym.wrappers.RecordVideo(env, "videos", episode_trigger=lambda x: x >= 0)
    options = {'init_type': 'run', 'file_name': 'test.pkl'}

    observation, info = env.reset(options=options)
    episodic_return = 0
    steps = 0
    for _ in range(1000):
        action = env.action_space.sample()
        steps += 1
        print(f"action: {action}")
        print(f"step: {steps}")
        observation, reward, done, truncated, info = env.step(action)
        if done:
            print(f"Episodic return: {info['episode']['r']}")
            observation, info = env.reset(options=options)
    env.close()