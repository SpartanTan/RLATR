from factory_env.envs.parameters import env_param
import gymnasium as gym
import matplotlib.pyplot as plt

plt.close('all')
params:env_param = env_param()
print(params.path_param.static_map)

env = gym.make('training-factory-v0', params=params,)
env.reset()
print(env.params.path_param.static_map)
fig, ax = plt.subplots(figsize=(10, 10))
# env.render()
env.path.render(ax)
# equal
plt.axis('equal')
plt.show()