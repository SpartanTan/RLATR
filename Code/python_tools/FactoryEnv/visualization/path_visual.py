from factory_env.envs.parameters import env_param
import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import time

# params = env_param()
# params.rangefinder = True
# params.sensor_param.resolution= np.deg2rad(3)
# env = gym.make('training-factory-v0', params=params,)
# env.reset()
# env.render()

# # env.path.render(ax)
# # equal
# plt.axis('equal')
# plt.show()

if __name__ == "__main__":
    params = env_param()
    params.rangefinder = True
    params.sensor_param.resolution= np.deg2rad(3)
    params.path_param.static_map = 0
    params.update()
    env = gym.make('training-factory-v0', params=params,)

    ## Start fake rangefinder
    params.rangefinder = False
    params.sensor_param.resolution= np.deg2rad(6)
    params.path_param.static_map = 0
    params.update()
    start_time = time.time()  # Start the timer
    for i in range(10):
        next_obs, info = env.reset()
        while True:
            loop_time = time.time()
            # plt.cla()
            action = env.action_space.sample()
            observation, reward, done, truncated, info = env.step(action)
            if done:
                env.render()
                # plt.show(block=True)
                break
            
            env.render()
            plt.pause(0.1)
    end_time = time.time()  # End the timer
    computational_time = end_time - start_time  # Calculate the computational time
    print(f"Computational Time: {computational_time} seconds")

    ## Start emulated rangefinder
    resolution_list = [12, 6, 3]
    params.rangefinder = True
    for resolution in resolution_list:
        params.sensor_param.resolution= np.deg2rad(resolution)
        params.update()
        start_time = time.time()  # Start the timer
        for i in range(10):
            next_obs, info = env.reset()
            while True:
                loop_time = time.time()
                action = env.action_space.sample()
                observation, reward, done, truncated, info = env.step(action)
                if done:
                    break
        end_time = time.time()  # End the timer
        computational_time = end_time - start_time  # Calculate the computational time
        print(f"Time for res: {resolution} is: {computational_time} seconds")