import numpy as np
import gymnasium as gym
from gymnasium import spaces
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.resolve()))

from factory_env.map.Nona2Grid import Nona2Grid
# from FactoryEnv.robot.ATR_RK4 import ATR_RK4
from factory_env.map.atr_path import ATRPath
from factory_env.map.Nona2Polygon import Nona2Polygon

np.seterr(all='ignore')
np.set_printoptions(precision=3, suppress=True)

class ATR_RK4():
    
    def __init__(self,
                 wheel_radius=0.125,
                 track_width=0.48,
                 init_state = np.array([0, 0, 0], dtype=float),
                 dt=0.1):
        self.wheel_radius = wheel_radius
        self.track_width = track_width
        self.state = init_state
        self.dt = dt
        
    def robot_pose_derivative(self, state:np.ndarray, wheel_speeds:list):
        x, y, theta = state
        left_wheel_speed, right_wheel_speed = wheel_speeds

        # Compute the linear and angular velocities
        linear_velocity = self.wheel_radius * (left_wheel_speed + right_wheel_speed) / 2
        angular_velocity = self.wheel_radius * \
            (right_wheel_speed - left_wheel_speed) / self.track_width

        # Calculate the change in x, y, and theta
        dx = linear_velocity * np.cos(theta)
        dy = linear_velocity * np.sin(theta)
        dtheta = angular_velocity

        return np.array([dx, dy, dtheta])

    def runge_kutta_4_step(self, wheel_speeds:list):
        """
        simulate the robot's movement
        
        ### Parameters
        - `wheel_speeds`: [wl, wr]. Note that the sequnece of input is [right, left], however the algorithm
        used here is [left, right], so have to switch the input first. 
        """
        # switch wheel speed sequence
        wheel_speeds = [wheel_speeds[1], wheel_speeds[0]]
        
        k1 = self.robot_pose_derivative(self.state, wheel_speeds)
        k2 = self.robot_pose_derivative(self.state + self.dt / 2 * k1, wheel_speeds)
        k3 = self.robot_pose_derivative(self.state + self.dt / 2 * k2, wheel_speeds)
        k4 = self.robot_pose_derivative(self.state + self.dt * k3, wheel_speeds)

        self.state = self.state + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        x_normalized = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        self.state[2] = x_normalized
        return np.copy(self.state)
    
    def set_position(self, position:np.ndarray):
        self.state = np.array([position[0], position[1], 0], dtype=float)
    
    def reset(self, position:np.ndarray):
        self.state = position
        
class FactoryEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4, 
                "path_name": ["OnePath", "testPath"]}
    
    def __init__(self,
                 map_dir:str, 
                 path_dir: str, 
                 path_name: str,
                 atr_init_position=None, 
                 render_mode=None):
        
        
        assert render_mode is None or render_mode in self.metadata["render_modes"]
        assert path_name in self.metadata["path_name"]
        self.render_mode = render_mode
        
        
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
        self.atrPath = ATRPath(path_dir=path_dir, path_name=path_name, if_render_text=False, path_type='pchip')

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
        self.start = self.atrPath.waypoints[0].copy()
        self.goal = self.atrPath.waypoints[-1].copy() # numpy array, (2,)
        self.wheel_radius = 0.125
        self.track_width = 0.48
        self.time_step = 0.05
        if atr_init_position is None:
            self.atr_init_state = np.array([self.start[0], self.start[1], 0], dtype=float)
        else:
            self.atr_init_state = np.array([atr_init_position[0], atr_init_position[1], 0], dtype=float)
            
        self.SIM_ATR = ATR_RK4(self.wheel_radius, self.track_width, self.atr_init_state, self.time_step)
        # self._agent_location_in_grid = np.array([0.0, 0.0])
        # self._target_location_in_grid = np.array([0.0, 0.0])

        self.observation_space = spaces.Dict(
            {
                "agent": spaces.Box(low=np.array([x_min, y_min, heading_min]), high=np.array([x_max, y_max, heading_max]), shape=(3,), dtype=np.float64),
                "target": spaces.Box(low=np.array([x_min, y_min]), high=np.array([x_max, y_max]), shape=(2,), dtype=np.float32),
                "is_it_dangerous": spaces.Discrete(2),
                "closest_point_on_wall": spaces.Box(low=np.array([x_min, y_min]), high=np.array([x_max, y_max]), shape=(2,), dtype=np.float64),
                "distance_to_wall": spaces.Box(low=0.0, high=10.0, shape=(1,), dtype=float),
            }
        )
        self.action_space = spaces.Box(low=0.0, high=5.0, shape=(2,), dtype=np.float32) # [wr wl]
        
    
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
        return {"agent": self.SIM_ATR.state, 
                "target": self.goal,
                "is_it_dangerous": self.danger,
                "closest_point_on_wall": self.closest_point_on_wall,
                "distance_to_wall": self.closest_distance_to_wall,}
    
    def _get_info(self) -> dict:
        """
        Helper function, return a dictionary, contains the manhattan distance between agent and target.

        ### Return
        - a dictionary contains the trajectory since the robot initialzied,
          a manhattan distance between agent and target.
        ### Examples
        >>> self._get_info()["distance"]
        {"distance": 4}
        """
        return {"trajectory": np.array(self.trajectory_), "distance_to_goal": np.linalg.norm(self.SIM_ATR.state[:2] - self.goal)}
    
    def reset(self, seed=None, options=None)->tuple:
        super().reset(seed=seed)
        
        self.closest_point_on_wall = np.array([0.0, 0.0], dtype=float)
        self.closest_distance_to_wall = 0.0
        self.danger = False
        self.SIM_ATR.reset(self.atr_init_state)
        self.trajectory_ = [self.atr_init_state]
        observation_ = self._get_obs()
        info_ = self._get_info()

        return observation_, info_

    def check_collision(self, state)->bool:
        """
        #### Parameters
        - `location`: a list, contains the location of the agent in real coordinate frame
        """
        ifcolli = self.polygon_map.check_if_collision(state[:2])
        return ifcolli

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
        atr_new_state = self.SIM_ATR.runge_kutta_4_step(action)
        if self.check_collision(atr_new_state):
            print("Collision!")
            atr_new_state = self.trajectory_[-1]
            terminated_ = True
            
        self.danger, self.closest_point_on_wall, self.closest_distance_to_wall = self.polygon_map.find_nearest_boundary_point(atr_new_state)
        
        self.trajectory_.append(atr_new_state)
        reward_ = 0.0
        terminated_ = False
        observation_ = self._get_obs()
        info_ = self._get_info()
        return observation_, reward_, terminated_, False, info_

    def render(self):
        if self.render_mode == "human":
            fig, ax = plt.subplots()
            self.polygon_map.render(ax)
            self.atrPath.render()
            
            # plot the atr position
            plt.scatter(self.SIM_ATR.state[0], self.SIM_ATR.state[1], s=100, marker='x', color='red')
            plt.axis("equal")
            plt.show()
        else:
            print("Render mode is not 'human', no rendering.")

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
    map_dir = "/home/zhicun/code/atr_rl/Code/python_tools/map/config/nona_description.json"
    path_dir = "/home/zhicun/code/atr_rl/Code/python_tools/map/config/nodes_tuve.json"
    path_name = "OnePath"
    atr_init_position = np.array([6.0, 6.0])
    env = FactoryEnv(map_dir, path_dir, path_name, atr_init_position, render_mode="human")
    observation, info = env.reset()
    # env.SIM_ATR.set_position(np.array([10.5, 3]))
    position = env._get_obs()["agent"]
    print(position)
    
    for i in range(100):
        action = np.array([2.0, 2.0])
        observation, reward, terminated, truncated, info = env.step(action)
    print(observation)
    print(info)
    observation, info  = env.reset()
    env.render()
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
    