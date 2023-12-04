import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from factory_env.envs.parameters import env_param


params:env_param = env_param()
params.path_param.No = 3
params.path_param.without_walls = True
params.rangefinder = False
params.update()
# Parallel environments
vec_env = make_vec_env("training-factory-v0", n_envs=4, params=params)

model = PPO("MlpPolicy", vec_env, verbose=1)
model.learn(total_timesteps=500000)
model.save("ppo_factory")

del model # remove to demonstrate saving and loading

model = PPO.load("ppo_factory")

obs = vec_env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = vec_env.step(action)
    vec_env.render("human")