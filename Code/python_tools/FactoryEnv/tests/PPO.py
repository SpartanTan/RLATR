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
    
    args = parser.parse_args()
    args.batch_size = int(args.num_envs * args.num_steps) # 4 * 300 = 1200
    args.minibatch_size = int(args.batch_size // args.num_minibatches) # 1200 // 6 = 200

    return args

class TimerThread(threading.Thread):
    def __init__(self):
        super(TimerThread, self).__init__()
        self.start_time = time.time()
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            elapsed_time = time.time() - self.start_time
            minutes = int(elapsed_time // 60)
            seconds = int(elapsed_time % 60)
            if elapsed_time % 10 <= 1:
                print(f"Elapsed time: {minutes} minutes, {seconds} seconds")
            time.sleep(1)

    def stop(self):
        self.stop_event.set()
        
def print_training_info(args, envs:gym.vector.SyncVectorEnv, agent, next_obs):
    print("***Training env information***")
    print(f"single action space: {envs.single_action_space.shape}") # (2,)
    print(f"next action shape [n_envs, n_actions]: {envs.action_space.shape}") # (n_envs, 2)
    print(f"single observation space: {envs.single_observation_space.shape}") # (26, )
    print(f"next observation shape [n_envs, n_obs]: {envs.observation_space.shape}") # (n_envs, 26)
    
    print(f"***NN agent info:***")
    print(agent)
    
    print(f"Rollout data(batch size) = num_envs * num_steps: {args.num_envs}*{args.num_steps}={args.batch_size}") # 4 * 300 = 1200
    print(f"num_updates = total_timesteps // batch_size: {args.total_timesteps} // {args.batch_size} = {args.total_timesteps // args.batch_size}") # 50000 // 1200 = 41
    print(f"mini batch size = batch_size // num_minibatches: {args.batch_size} // {args.num_minibatches} = {args.minibatch_size}") # 1200 // 6 = 200
    
    print(f"agent get value shape [n_envs, 1]: {agent.get_value(next_obs).shape}")
    print(f'agent get action and value shape [n_envs, n_actions]: {agent.get_action_and_value(next_obs)[0].shape}')
    
def class_to_dict(obj):
    output = {}
    for attr_name, attr_value in vars(obj).items():
        if isinstance(attr_value, (int, float)):
            output[attr_name] = attr_value
        elif hasattr(attr_value, '__dict__'):
            nested_dict = class_to_dict(attr_value)
            output = {**output, **nested_dict} # Merge dictionaries
    return output

def log_params(env_params_instance, writer):
    """
    This function logs all environment parameters to tensorboard
    Parameters are from parameters.py
    """
    params = vars(env_params_instance)
    writer.add_text(
        "Environment parameters",
        "|param|value|\n|-|-|\n%s" %("\n".join([f"|{sub_key}|{sub_value}|" for sub_key, sub_value in params.items() if isinstance(sub_value, (int, float)) ])),
    )
    for key, value in params.items():
        if isinstance(value, path_param):
            writer.add_text(
                "Path parameters",
                "|param|value|\n|-|-|\n%s" %("\n".join([f"|{sub_key}|{sub_value}|" for sub_key, sub_value in vars(value).items()])),
            )
        if isinstance(value, ATR_param):
            writer.add_text(
                "ATR parameters",
                "|param|value|\n|-|-|\n%s" %("\n".join([f"|{sub_key}|{sub_value}|" for sub_key, sub_value in vars(value).items()])),
            )
        if isinstance(value, reward_param):
            writer.add_text(
                "Reward parameters",
                "|param|value|\n|-|-|\n%s" %("\n".join([f"|{sub_key}|{sub_value}|" for sub_key, sub_value in vars(value).items()])),
            )
        if isinstance(value, sensor_param):
            writer.add_text(
                "Sensor parameters",
                # "|param|value|\n|-|-|\n%s" %("\n".join([f"|{sub_key}|{sub_value}|" for sub_key, sub_value in vars(value).items() if sub_key == "distance_threshold" or "sensor_angle" or "nsectors" or "num_points_on_walls" or "num_points_on_obstacles"])),
                "|param|value|\n|-|-|\n%s" %("\n".join([f"|{sub_key}|{sub_value}|" for sub_key, sub_value in vars(value).items() if isinstance(sub_value, (int, float)) ])),
            )
    
def check_param(params: env_param):
    params.full_plot = True
    print(f"Environment setup: \n\
            Path length: {params.path_param.Lp} m\n\
            Number of obstacles: {params.path_param.Nw}\n\
            ATR max velocity: {params.atr_param.atr_linear_vel_max} m/s\n\
            ATR max wheel speed: {params.atr_param.atr_w_max} rad/s\n\
            target finishing time: {params.path_param.target_finishing_time} s\n\
            step time: {params.atr_param.dt} s\n\
            ")
    assert params.path_param.target_finishing_time >= params.path_param.Lp / params.atr_param.atr_linear_vel_max, "The target finishing time is too short."


def make_env(gym_id, params, seed, idx, capture_video, run_name):
    def thunk():
        env = gym.make(gym_id, params=params, render_mode="rgb_array")
        env = gym.wrappers.RecordEpisodeStatistics(env)
        if capture_video:
            if idx == 0:
                env = gym.wrappers.RecordVideo(env, f"videos/{run_name}", episode_trigger=lambda x: x % 30 == 0)
        env = gym.wrappers.ClipAction(env)
        env = gym.wrappers.NormalizeObservation(env) # standarlization
        env = gym.wrappers.TransformObservation(env, lambda obs: np.clip(obs, -10, 10))
        env = gym.wrappers.NormalizeReward(env)
        env = gym.wrappers.TransformReward(env, lambda reward: np.clip(reward, -10, 10))
        
        env.action_space.seed(seed)
        env.observation_space.seed(seed)
        return env
    return thunk

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer

class Agent(nn.Module):
    def __init__(self):
        super().__init__()
        self.critic = nn.Sequential(
               layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 64)), # input feature: 26, output feature: 64
               nn.Tanh(),
               layer_init(nn.Linear(64, 64)),
               nn.Tanh(),
               layer_init(nn.Linear(64, 1), std=1.0)
          )
        self.actor_means = nn.Sequential(
            layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, np.prod(envs.single_action_space.shape)), std=0.01) # 0.01 make the probability of taking each action be similar
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, np.prod(envs.single_action_space.shape))) # 2-dim vector, each dim is the logstd for each action
        
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
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = Normal(action_mean, action_std)
        if action is None:
            action = probs.sample()
        return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.critic(x)
                
def extract_info(data):
    r"""
    This function can extract the items in `info` variable returned by envs.step(). \  
    Since we use RecordEpisodeStatistics, every time after there is one env done, the info will store the episode information. \ 
    e.g. \ 
    info = {'final_observation': np.array([np.array([ 0.172, -0.03 ,  0.092,  0.137, -0.036, -0.161,  0.   ,  0.   ,
               0.   ,  0.873,  0.807,  0.807,  0.834,  0.807,  0.807,  0.807,
               0.803,  0.803,  0.803,  0.803,  0.803,  0.803,  0.   ,  0.873,
               0.   ,  0.   ], dtype=np.float32), None],dtype=object), 
            '_final_observation': np.array([ True, False]), \ 
            'final_info': np.array([{'episode': {'r': np.array([-7931.417],dtype=np.float32), 'l': np.array([45], dtype=np.int32), 't': np.array([0.094], dtype=np.float32)}}, 
                                    {'episode': {'r': np.array([-8000.], dtype=np.float32), 'l': np.array([45], dtype=np.int32), 't': np.array([0.094], dtype=np.float32)}}],dtype=object),
            '_final_info': np.array([ True, False])}
    This function therefore returns a numpy array for each key.
    
    ### Example
    >>> final_observation, _final_observation, final_info, _final_info = extract_info(info)
    """
    # Extract arrays from dictionary
    final_observation = data['final_observation']
    _final_observation = data['_final_observation']
    final_info = data['final_info']
    _final_info = data['_final_info']
    # Convert numpy bool to python bool

    return final_observation, _final_observation, final_info, _final_info

def extract_episode_r_l_t(final_info):
    """
    This function can extract the episode information from the final_info array. \ 
    If there are more than one element in the array, e.g. \ 
    final_info = np.array([{'episode': {'r': np.array([-7931.417],dtype=np.float32), 'l': np.array([45], dtype=np.int32), 't': np.array([0.094], dtype=np.float32)}}, 
                                    {'episode': {'r': np.array([-8000.], dtype=np.float32), 'l': np.array([45], dtype=np.int32), 't': np.array([0.094], dtype=np.float32)}}],dtype=object)
    It will return a list containing all the r, l, t values. \
    
    ### Parameters
    `final_info` is extracted from `extract_info` function.
             
    ### Returns
    `r_values` [n_episodes] \ 
    `l_values` [n_episodes] \ 
    `t_values` [n_episodes]  

    ### Example
    >>> r_values, l_values, t_values = extract_episode_r_l_t(final_info)
    >>> print(r_values)
    >>> [-7931.417, -8000.]                             
    """
    r_values = [episode['episode']['r'][0] for episode in final_info if episode is not None]
    l_values = [episode['episode']['l'][0] for episode in final_info if episode is not None]
    t_values = [episode['episode']['t'][0] for episode in final_info if episode is not None]
    return r_values, l_values, t_values  
       
if __name__ == "__main__":
    start_time = time.time()
    timer = TimerThread()
    timer.start()
    
    args = parse_args()
    params = env_param()
    check_param(params)
    
    run_name = f"{args.gym_id}_{args.exp_name}_{args.seed}_{int(time.time())}"
    
    """
 
       _                                 _                         _    ___                             _ _                _               
      | |_ ___  ___ _ __  ___  ___  _ __| |__   ___   __ _ _ __ __| |  ( _ )   __      ____ _ _ __   __| | |__    ___  ___| |_ _   _ _ __  
      | __/ _ \/ __| '_ \/ __|/ _ \| '__| '_ \ / _ \ / _` | '__/ _` |  / _ \/\ \ \ /\ / / _` | '_ \ / _` | '_ \  / __|/ _ \ __| | | | '_ \ 
      | ||  __/\__ \ | | \__ \ (_) | |  | |_) | (_) | (_| | | | (_| | | (_>  <  \ V  V / (_| | | | | (_| | |_) | \__ \  __/ |_| |_| | |_) |
       \__\___||___/_| |_|___/\___/|_|  |_.__/ \___/ \__,_|_|  \__,_|  \___/\/   \_/\_/ \__,_|_| |_|\__,_|_.__/  |___/\___|\__|\__,_| .__/ 
                                                                                                                                    |_|    
 
    """
    if args.track:
        import wandb
        wandb.init(
            project=args.wandb_project_name,
            entity=args.wandb_entity,
            sync_tensorboard=True,
            config={**vars(args), **class_to_dict(params)},
            name=run_name,
            monitor_gym=True,
            save_code=True,
        )
    writer = SummaryWriter(f"runs/{run_name}")
    writer.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n%s" %("\n".join([f"|{key}|{value}|" for key, value in vars(args).items()])),
    )
    log_params(params, writer)
    
    """
 
                         _   _       _ _   
       ___  ___  ___  __| | (_)_ __ (_) |_ 
      / __|/ _ \/ _ \/ _` | | | '_ \| | __|
      \__ \  __/  __/ (_| | | | | | | | |_ 
      |___/\___|\___|\__,_| |_|_| |_|_|\__|
                                           
 
    """
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.backends.cudnn.deterministic=args.torch_deterministic
    device = torch.device("cuda" if torch.cuda.is_available() and args.cuda else "cpu")
    
    """
 
       _____ _   ___     __  _____ _____ ____ _____ 
      | ____| \ | \ \   / / |_   _| ____/ ___|_   _|
      |  _| |  \| |\ \ / /    | | |  _| \___ \ | |  
      | |___| |\  | \ V /     | | | |___ ___) || |  
      |_____|_| \_|  \_/      |_| |_____|____/ |_|  
                                                    
 
    """
    # env = gym.make("training-factory-v0", params=params, render_mode="rgb_array")
    # env = gym.wrappers.RecordEpisodeStatistics(env)
    # # env = gym.wrappers.RecordVideo(env, "videos", episode_trigger=lambda x: x >= 0)
    # options = {'init_type': 'run', 'file_name': 'test.pkl'}

    # observation, info = env.reset(options=options)
    # episodic_return = 0
    # for _ in range(1000):
    #     action = env.action_space.sample()
    #     observation, reward, done, truncated, info = env.step(action)
    #     if done:
    #         print(f"Episodic return: {info['episode']['r']}")
    #         observation, info = env.reset(options=options)
    # env.close()
    
    # envs = gym.vector.AsyncVectorEnv([make_env(args.gym_id, args.seed+i, i, args.capture_video, run_name) for i in range(1)])
    # envs = gym.vector.SyncVectorEnv([make_env(args.gym_id, params) for i in range(2)])
    # envs = gym.vector.SyncVectorEnv([make_env(args.gym_id, params, args.seed, i, args.capture_video, run_name) for i in range(args.num_envs)])
    
    # #  Vectorized ENV testing code
    # observation, info = envs.reset()
    # for _ in range(2000):
    #     action = envs.action_space.sample()
    #     observation, reward, done, truncated, info = envs.step(action)
        
    #     if not info=={}:
    #         final_observation, _final_observation, final_info, _final_info = extract_info(info)
    #         r_values, l_values, t_values= extract_episode_r_l_t(final_info)
    #         print(f"Episodic return: {r_values}")
    # raise Exception("end here")

    """
 
       _____          _       _                                __ _       
      |_   _| __ __ _(_)_ __ (_)_ __   __ _    ___ ___  _ __  / _(_) __ _ 
        | || '__/ _` | | '_ \| | '_ \ / _` |  / __/ _ \| '_ \| |_| |/ _` |
        | || | | (_| | | | | | | | | | (_| | | (_| (_) | | | |  _| | (_| |
        |_||_|  \__,_|_|_| |_|_|_| |_|\__, |  \___\___/|_| |_|_| |_|\__, |
                                      |___/                         |___/ 
 
    """
    
    envs = gym.vector.SyncVectorEnv([make_env(args.gym_id, params, args.seed, i, args.capture_video, run_name) for i in range(args.num_envs)])
    assert isinstance(envs.single_observation_space, gym.spaces.Box), "only continuous action space is supported"
    
    agent:Agent = Agent().to(device)
    # agent.load_state_dict(torch.load("agents/training-factory-v0_PPO_1_1688932490.pth"))
    optimizer = optim.Adam(agent.parameters(), lr=args.learning_rate, eps=1e-5)
    obs = torch.zeros((args.num_steps, args.num_envs) + envs.single_observation_space.shape).to(device)   # [n_steps, n_envs, n_obs]
    actions = torch.zeros((args.num_steps, args.num_envs) + envs.single_action_space.shape).to(device)    # [n_steps, n_envs, n_actions]
    logprobs = torch.zeros((args.num_steps, args.num_envs)).to(device)                                    # [n_steps, n_envs]
    rewards = torch.zeros((args.num_steps, args.num_envs)).to(device)                                     # [n_steps, n_envs]
    dones = torch.zeros((args.num_steps, args.num_envs)).to(device)  # [n_steps, n_envs] stores whether this step is done. Will be converted to int in GAE
    values = torch.zeros((args.num_steps, args.num_envs)).to(device) # [n_steps, n_envs] stores V(s)
    
    
    global_step = 0
    start_time = time.time()
    observation, info = envs.reset()
    next_obs = torch.Tensor(observation).to(device) # [4, 26], 4 env, 26 obs
    next_done = torch.zeros(args.num_envs).to(device)
    num_updates = args.total_timesteps // args.batch_size
    
    print_training_info(args, envs, agent, next_obs)
    
    for update in range(1, num_updates + 1):
        if args.anneal_lr:
            frac = 1.0 - (update - 1.0) / num_updates
            lrnow = args.learning_rate * frac
            optimizer.param_groups[0]["lr"] = lrnow
        
        for step in range(0, args.num_steps):
            global_step += 1 * args.num_envs # 4 envs steps together
            obs[step] = next_obs
            dones[step] = next_done
            
            # ALGO LOGIC: action logic
            with torch.no_grad():
                action, logprob, _, value = agent.get_action_and_value(next_obs)
                values[step] = value.flatten()
            actions[step] = action
            logprobs[step] = logprob
            
            # Step the env
            next_obs, reward, done, truncated, info = envs.step(action.cpu().numpy())
            if np.any(np.isnan(next_obs)):
                print(next_obs)
                raise ValueError("next_obs is nan")
            rewards[step] = torch.tensor(reward).to(device).view(-1)
            next_obs, next_done = torch.Tensor(next_obs).to(device), torch.Tensor(done).to(device)
            
            if not info=={}:
                final_observation, _final_observation, final_info, _final_info = extract_info(info)
                ep_r, ep_l, ep_t = extract_episode_r_l_t(final_info)
                print(f"global_step={global_step}, episodic_return: {ep_r}")
                writer.add_scalar("charts/episodic_return", np.mean(ep_r), global_step)
                writer.add_scalar("charts/episodic_length", np.mean(ep_l), global_step)
        
        with torch.no_grad():
            # This is the value for the observation after last step. V(300)
            # The next_obs here won't be stored in obs array, neither this next_value
            # The next_value here is for A(299) = r(299) + gamma * V(300) - V(299)
            next_value = agent.get_value(next_obs).reshape(1, -1) # next_obs: [n_envs, n_obs], get_value(): [n_envs, 1], next_value: [1, n_envs]
            if args.gae:
                advantages = torch.zeros_like(rewards).to(device) # [num_steps, n_envs]
                lastgaelam = 0
                for t in reversed(range(args.num_steps)):
                    # For the last step
                    # A(299) = r(299) + gamma * V(300) - V(299)
                    # Notice that the "done" episode 
                    if t == args.num_steps - 1:
                        next_done = next_done.to(torch.int)
                        nextnonterminal = 1 - next_done # we need the terms that done is False. Thus reverse it.
                        nextvalues = next_value # V(300)
                    else:
                        nextnonterminal = 1 - dones[t+1]
                        nextvalues = values[t+1] # V(299), V(298), ...
                    delta = rewards[t] + args.gamma * nextvalues * nextnonterminal - values[t]
                    advantages[t] = lastgaelam = delta + args.gamma * args.gae_lambda * nextnonterminal * lastgaelam
                returns = advantages + values
            else:
                # Using regular method
                returns = torch.zeros_like(rewards).to(device)
                for t in reversed(range(args.num_steps)):
                    if t == args.num_steps - 1:
                        nextnonterminal = 1.0 - next_done
                        next_return = next_value
                    else:
                        nextnonterminal = 1.0 - dones[t+1]
                        next_return = returns[t+1]
                    returns[t] = rewards[t] + args.gamma * nextnonterminal * next_return # regular return: sum of discounted rewards
                advantages = returns - values  # Q-V
                
        # flatten the batch
        b_obs = obs.reshape(-1, *obs.shape[2:]) # [300,4, 26] -> [1200, 26]. 
        b_logprobs = logprobs.reshape(-1) # [1200]
        b_actions = actions.reshape((-1,) + envs.single_action_space.shape) # [n_steps, n_envs, n_actions] -> [1200, 2]
        b_advantages = advantages.reshape(-1) # [1200]
        b_returns = returns.reshape(-1) # [1200]
        b_values = values.reshape(-1) # [1200]
        b_actions.long()
        b_inds = np.arange(args.batch_size) # indices from 0 to 1200
        clipfracs = []
        
        for epoch in range(args.update_epochs): # training on the current dataset for update_epoches times (currently it's 4)
            np.random.shuffle(b_inds) # shuffle the indices. Train with different data every time
            for start in range(0, args.batch_size, args.minibatch_size): # each iteration takes minibatch_size(200) data from the dataset (1200)
                end = start + args.minibatch_size # index for the end of the this minibatch data [[0, 199], [200, 399], ...]
                mb_inds = b_inds[start:end] # extract the data indices. This contains `minibatch_size` indices
                # feed NN with observation[minibatch_size, 26] and actions [minibatch_size, 2]
                _, newlogproba, entropy, newvalues = agent.get_action_and_value(b_obs[mb_inds], b_actions[mb_inds]) 
                # The 'newvalues' will be different from the values in 'values' array we got from above because
                # the values in 'values' are all from the same NN
                # However the 'newvalues' here are from NN after DG by each minibatch
                # Thus different.
                
                logratio = newlogproba - b_logprobs[mb_inds] # log(P(new)/P(old)) = log(P(new)) - log(P(old))
                ratio = logratio.exp() # r = P(new) / P(old)
                
                with torch.no_grad():
                    # KL[old, new] = log(old/new)
                    # but we have r = P(new) / P(old)
                    # since log(old/new) = - log(new/old) 
                    old_approx_kl = (-logratio).mean()
                    # http://joschu.net/blog/kl-approx.html Approximating KL Divergence
                    # r = new/old
                    # KL[old, new] = (r - 1) - log(r)
                    approx_kl = ((ratio - 1.0) - logratio).mean()
                    clipfracs += [((ratio - 1.0).abs() > args.clip_coef).float().mean().item()] # how often the clip objective is triggered
                
                mb_advantages = b_advantages[mb_inds]
                if args.norm_adv:
                    mb_advantages = (mb_advantages - mb_advantages.mean()) / (mb_advantages.std() + 1e-8) # x:= (x-mu)/sigma
                
                # Policy loss
                pg_loss1 = -mb_advantages * ratio
                pg_loss2 = -mb_advantages * torch.clamp(ratio, 1.0 - args.clip_coef, 1.0 + args.clip_coef)
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()
        
                # Value loss
                # unclipped loss: L^VF = E[(V - V_target)^2]
                # clipped V: clip(V-V_target, -coef, coef) + V_target
                # clipped loss: (v_clipped - V_target)^2
                newvalues = newvalues.view(-1) # size: [minibatch_size]->[128] e.g. [1.1 -2.3 -1.3 ...]
                newvalues = newvalues.view(-1) # (minibatch_size ,)
                if args.clip_vloss:
                    v_loss_unclipped = (newvalues - b_returns[mb_inds]) ** 2
                    v_clipped = b_values[mb_inds] + torch.clamp(newvalues - b_values[mb_inds], -args.clip_coef, args.clip_coef)
                    v_loss_clipped = (v_clipped - b_returns[mb_inds]) ** 2
                    v_loss_max = torch.max(v_loss_unclipped, v_loss_clipped)
                    v_loss = 0.5 * v_loss_max.mean()
                else:
                    v_loss = 0.5 * ((newvalues - b_returns[mb_inds]) ** 2).mean()  # the factor 0.5 here is for simplifying the derivitive. d(0.5*(x-a)^2) -> (x-a)
                
                # Entropy loss
                entropy_loss = entropy.mean()
                loss = pg_loss + args.vf_coef * v_loss - args.ent_coef * entropy_loss 

                # Optimize 
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), args.max_grad_norm)
                optimizer.step()
            
            # early breaking at batch level. Also possible to implenment this in minibatch level
            if args.target_kl is not None:
                if approx_kl > args.target_kl:
                    break
                
        y_pred, y_true = b_values.cpu().numpy(), b_returns.cpu().numpy()
        var_y = np.var(y_true)
        explained_var = np.nan if var_y == 0 else 1 - np.var(y_true - y_pred) / var_y # tells if value function is a good indicator of the returns
        
        writer.add_scalar("charts/learning_rate", optimizer.param_groups[0]['lr'], global_step)
        writer.add_scalar("losses/value_loss", v_loss.item(), global_step) # .item() take the scalar value out of the tensor
        writer.add_scalar("losses/policy_loss", pg_loss.item(), global_step)
        writer.add_scalar("losses/entropy", entropy_loss.item(), global_step)
        writer.add_scalar("losses/approx_kl", approx_kl.item(), global_step)
        writer.add_scalar("losses/clipfrac", np.mean(clipfracs), global_step)
        writer.add_scalar("debug/explained_variance", explained_var, global_step)
        writer.add_scalar("debug/advantage_mean", advantages.mean().item(), global_step)
        print("SPS:", int(global_step / (time.time() - start_time)))
        print(f"Time spent so far: {int((time.time() - start_time) // 60)} miniutes, {int((time.time() - start_time) % 60)} seconds")
        writer.add_scalar("charts/SPS", int(global_step / (time.time() - start_time)), global_step)
    
    torch.save(agent.state_dict(), f"agents/{run_name}.pth")
    print(f"Training time: {int((time.time() - start_time) // 60)} miniutes, {int((time.time() - start_time) % 60)} seconds")
    print(f"NN parameters saved to agents/{run_name}.pth")
    
    envs.close()
    writer.close()
    timer.stop()
    timer.join()