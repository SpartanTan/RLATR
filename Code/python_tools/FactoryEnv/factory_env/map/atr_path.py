import json
import numpy as np  
import math
import matplotlib.pyplot as plt
import scipy.interpolate as interp

# from cubic_spline import Spline2D
from scipy.interpolate import CubicSpline
np.seterr(all='ignore')
np.set_printoptions(precision=3, suppress=True)

class ATRPath():
    """
    ### Parameters
    - `path_dir`: str, the path of the json file, default: "./config/nodes_tuve.json"
    - `path_type`: str, the type of the path, default: "spline", options: ["spline", "straight", "pchip"]
    - `trajectory_point_interval`: float, the interval of the trajectory points, default: 0.1
    - `if_render_text`: bool, if render the text of the waypoints, default: True
    
    ### Example
    ```python
    atrPath = ATRPath()
    >>> data_synthesis(4, 282): | inteploated_x |
                                | inteploated_y |
                                | trajectory_lengths |
                                | yaw_angles |
    >>> shape of data_synthesis: (4, 282)
    >>> shape of waypoints_x: (14,)
    >>> shape of waypoints(14, 2)
    >>> shape of even_trajectory: (282, 2)
    >>> shape of yaw_angles: (282,)
    ```
    """
    metadata = {"path_type": ["spline", "straight", "pchip"]}
    def __init__(self, 
                 path_dir: str = "./config/nodes_tuve.json",
                 path_name: str = "OnePath",
                 path_type: str = "spline",
                 trajectory_point_interval = 0.1,
                 if_render_text=True,
                 ):
        
        assert path_type in self.metadata["path_type"]
        
        self.path_dir = path_dir
        self.path_type = path_type
        self.trajectory_point_interval = trajectory_point_interval
        self.if_render_text = if_render_text
        self.data_synthesis = None
        
        with open(self.path_dir, "r") as file:
            # Load the JSON content into a Python dictionary
            data = json.load(file)

            self.paths = data[path_name]
            self.nodes = data["Nodes"]
            self.path_waypoints = []
        self.construct_waypoints()
        if self.path_type == 'spline':
            self.generate_spline_trajectory()
        elif self.path_type == 'straight':
            self.generate_straight_trajectory()
        elif self.path_type == 'pchip':
            self.generate_pchip_trajectory()  

        self.synthesis()
    
    def construct_waypoints(self):
        node_positions = {node['id']: np.array(node['position'][:2]) for node in self.nodes}

        for path in self.paths:
            seq = 0
            self.path_id = path['id']
            self.node_ids = path['graph']
            self.path_waypoints = [node_positions[node_id] for node_id in self.node_ids]
            if(self.if_render_text):
                for wp in self.path_waypoints:
                    seq += 1
                    plt.text(wp[0], wp[1], f'{seq}, {wp}', fontsize=8)
            self.path_waypoints = np.array(self.path_waypoints)
            self.waypoints_x = self.path_waypoints[:, 0]
            self.waypoints_y = self.path_waypoints[:, 1]
        self.waypoints = np.column_stack((self.waypoints_x, self.waypoints_y))
        
    def generate_spline_trajectory(self):
        # Calculate the arc length
        ds = np.sqrt(np.diff(self.waypoints_x)**2 + np.diff(self.waypoints_y)**2)
        s = np.hstack(([0], np.cumsum(ds)))

        # Create the cubic spline using the arc length as the parameter
        cs = CubicSpline(s, self.waypoints_y, bc_type='clamped')
        cs_x = CubicSpline(s, self.waypoints_x, bc_type='clamped')

        # Calculate the total trajectory length and number of points for interpolation
        total_length = s[-1]
        
        num_points = int(total_length / self.trajectory_point_interval ) + 1
        # Generate interpolation points
        self.interp_trajectory_lengths = np.linspace(s[0], s[-1], num=num_points)
        self.interpolated_y = cs(self.interp_trajectory_lengths)
        self.interpolated_x = cs_x(self.interp_trajectory_lengths)

        # Calculate the yaw angle using the derivatives of the cubic spline
        dx = cs_x.derivative(1)(self.interp_trajectory_lengths)
        dy = cs.derivative(1)(self.interp_trajectory_lengths)
        self.yaw_angles = np.arctan2(dy, dx)
        self.yaw_angles[0] = self.yaw_angles[1]
        self.yaw_angles[-1] = self.yaw_angles[-2]
        
    
    def generate_straight_trajectory(self):
        
        interpolated_points = [self.waypoints[0]]
        for i in range(1, len(self.waypoints)):
            dist = np.linalg.norm(self.waypoints[i] - self.waypoints[i-1])
            num_points = int(np.ceil(dist / self.trajectory_point_interval))
            t_values = np.linspace(0, 1, num=num_points)
            
            for t in t_values:
                point = self.waypoints[i-1] + t * (self.waypoints[i] - self.waypoints[i-1])
                if not np.allclose(point, interpolated_points[-1]):
                    interpolated_points.append(point)
        interpolated_points = np.array(interpolated_points)
        
        delta_points = np.diff(interpolated_points, axis=0)
        segment_distances = np.linalg.norm(delta_points, axis=1)
        cumulative_distances = np.cumsum(segment_distances)
        self.interp_trajectory_lengths = np.insert(cumulative_distances, 0, 0)  # Add a zero at the beginning for the starting point
    
        self.yaw_angles = np.arctan2(delta_points[:, 1], delta_points[:, 0])
        self.yaw_angles = np.append(self.yaw_angles, self.yaw_angles[-1])
        self.interpolated_x = np.squeeze(interpolated_points[:, 0])
        self.interpolated_y = np.squeeze(interpolated_points[:, 1])
    
    def generate_pchip_trajectory(self):
        def arc_length(path, t, dt):
            derivative = path.derivative()(t)
            return np.sqrt(np.sum(derivative**2, axis=1)) * dt
        
        # waypoints = np.column_stack((x, y))
        path = interp.PchipInterpolator(np.linspace(0, 1, len(self.waypoints)), self.waypoints, axis=0)
        # Calculate even trajectory points
        t = np.linspace(0, 1, 100)
        dt = t[1] - t[0]
        cumulative_arclength = np.cumsum(arc_length(path, t, dt))
        total_arclength = cumulative_arclength[-1]
        num_points = int(np.ceil(total_arclength / self.trajectory_point_interval))
        even_t = np.zeros(num_points)
        current_arclength = 0
        for i in range(1, num_points):
            current_arclength += self.trajectory_point_interval
            even_t[i] = np.interp(current_arclength, cumulative_arclength, t)
        even_trajectory = path(even_t)
        
        # calculate yaw angles at each point
        even_arclength = np.interp(even_t, t, cumulative_arclength)
        # Calculate yaw angles at each point
        path_derivative = path.derivative()(even_t)
        yaw_angles = np.arctan2(path_derivative[:, 1], path_derivative[:, 0])

        self.interp_trajectory_lengths = even_arclength
        self.yaw_angles = yaw_angles
        # self.yaw_angles = np.append(self.yaw_angles, self.yaw_angles[-1])
        self.interpolated_x = np.squeeze(even_trajectory[:, 0])
        self.interpolated_y = np.squeeze(even_trajectory[:, 1])
        
    def synthesis(self):
        """
        Stack the data together, into a shape of (4, num_points)
        | inteploated_x | inteploated_y | trajectory_lengths | yaw_angles |
        """
        self.data_synthesis = np.vstack((self.interpolated_x, self.interpolated_y, self.interp_trajectory_lengths ,self.yaw_angles))
        self.even_trajectory = np.column_stack((self.interpolated_x, self.interpolated_y))
        
    def render(self):
        # ploat waypoints
        plt.plot(self.waypoints_x, self.waypoints_y, marker='o',
                     markersize=5, linestyle='-', linewidth=0.2, label=f"Path {self.path_id}")
        # plot the interpolated points
        plt.scatter(self.even_trajectory[:, 0], self.even_trajectory[:,1], s=0.5, marker='.', label='Interpolated Points')
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("s[m]")
        plt.ylabel("y[m]")
        plt.legend()
        
        # fig, ax = plt.subplots(1)
        # plt.plot(self.data_synthesis[2,:], self.data_synthesis[3,: ], 'b-', label="Yaw angle")
    
    def print_info(self):
        print(f"shape of data_synthesis : {self.data_synthesis.shape}")
        print(f"shape of waypoints_x: {self.waypoints_x.shape}")
        print(f"shape of waypoints (numpy array, (10,2)): {self.waypoints.shape}")
        print(f"total length of the trajectory: {round(self.data_synthesis[2,-1], 3)}")
        print(f"shape of even_trajectory: {self.even_trajectory.shape}")
        print(f"shape of yaw_angles: {self.yaw_angles.shape}")
        
if __name__ == "__main__":
    # test_spline2d()
    path_dir = "/home/zhicun/code/atr_rl/Code/python_tools/FactoryEnv/factory_env/map/config/nodes_tuve.json"
    path_name = "testPath"
    atrPath = ATRPath(path_dir=path_dir, path_name=path_name, if_render_text=True, path_type='pchip')
    atrPath.render()
    atrPath.print_info()
    plt.show()
    
    
    