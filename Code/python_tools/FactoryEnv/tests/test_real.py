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

params:env_param = env_param()
map_dir = "/home/zhicun/code/atr_rl/Code/python_tools/FactoryEnv/factory_env/map/config/nona_description.json"
path_dir = "/home/zhicun/code/atr_rl/Code/python_tools/FactoryEnv/factory_env/map/config/nodes_tuve.json"
path_name = "OnePath"
# env = gym.make('training-factory-v0', params=params,)
env = gym.make('real-factory-v0', params=params, map_dir=map_dir, path_dir=path_dir, path_name=path_name)
observation, info = env.reset()
env.step(np.array([0.0, 0.0]))
env.render()
