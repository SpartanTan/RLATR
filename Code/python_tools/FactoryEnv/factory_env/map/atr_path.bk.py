import json
import numpy as np  
import math
import matplotlib.pyplot as plt

# from cubic_spline import Spline2D
from scipy.interpolate import CubicSpline
np.seterr(all='ignore')
np.set_printoptions(precision=3, suppress=True)

class ATRPath():
    metadata = {"path_type": ["spline", "straight"]}
    def __init__(self, 
                 path_dir: str = "./config/nodes_tuve.json",
                 path_type: str = "spline",
                 constance_distance: float = 0.1,
                 if_render_text=True,
                 ):
        
        assert path_type in self.metadata["path_type"]
        
        self.path_dir = path_dir
        self.path_type = path_type
        self.constance_distance = constance_distance
        self.if_render_text = if_render_text
        self.data_synthesis = None
        
        with open(self.path_dir, "r") as file:
            # Load the JSON content into a Python dictionary
            data = json.load(file)

            self.paths = data["OnePath"]
            self.nodes = data["Nodes"]
            self.path_waypoints = []
        self.construct_waypoints()
        if self.path_type == 'spline':
            self.generate_spline_trajectory()
        elif self.path_type == 'straight':
            self.generate_straight_trajectory()    

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
                        
    def generate_spline_trajectory(self):
        # Calculate the arc length
        ds = np.sqrt(np.diff(self.waypoints_x)**2 + np.diff(self.waypoints_y)**2)
        s = np.hstack(([0], np.cumsum(ds)))

        # Create the cubic spline using the arc length as the parameter
        cs = CubicSpline(s, self.waypoints_y, bc_type='clamped')
        cs_x = CubicSpline(s, self.waypoints_x, bc_type='clamped')

        # Calculate the total trajectory length and number of points for interpolation
        total_length = s[-1]
        
        num_points = int(total_length / self.constance_distance ) + 1
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
        
        points = np.column_stack((self.waypoints_x, self.waypoints_y))
        total_distance = np.sum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
        num_interp_points = (total_distance // self.constance_distance).astype(int) + 1
        interp_points = [points[0]]
        interp_yaw_angles = []
        self.interp_trajectory_lengths = [0]

        current_distance = 0
        current_point_index = 0

        for _ in range(num_interp_points - 1):
            current_distance += self.constance_distance
            while current_point_index < len(points) - 1:
                segment_distance = np.sqrt(np.sum((points[current_point_index + 1] - points[current_point_index])**2))
                if current_distance < segment_distance:
                    break
                else:
                    current_distance -= segment_distance
                    current_point_index += 1

            t = current_distance / segment_distance
            new_point = points[current_point_index] + t * (points[current_point_index + 1] - points[current_point_index])
            distance = np.linalg.norm(new_point - interp_points[-1])
            interp_points.append(new_point)

            dx = new_point[0] - interp_points[-2][0]
            dy = new_point[1] - interp_points[-2][1]
            yaw_angle = np.arctan2(dy, dx)
            interp_yaw_angles.append(yaw_angle)
            self.interp_trajectory_lengths.append(self.interp_trajectory_lengths[-1] + distance)

        interp_yaw_angles.append(interp_yaw_angles[-1]) # add one angle for the last point
        interp_points = np.array(interp_points)
        self.interpolated_x = interp_points[:, 0]
        self.interpolated_y = interp_points[:, 1]
        self.yaw_angles = np.array(interp_yaw_angles)
        self.interp_trajectory_lengths = np.array(self.interp_trajectory_lengths)
    
    
    def synthesis(self):
        """
        Stack the data together, into a shape of (4, num_points)
        | inteploated_x | inteploated_y | trajectory_lengths | yaw_angles |
        """
        self.data_synthesis = np.vstack((self.interpolated_x, self.interpolated_y, self.interp_trajectory_lengths ,self.yaw_angles))
        
    def render(self):
        # ploat waypoints
        plt.plot(self.waypoints_x, self.waypoints_y, marker='o',
                     markersize=5, linestyle='-', linewidth=1, label=f"Path {self.path_id}")
        # plot the interpolated points
        plt.plot(self.data_synthesis[0,:], self.data_synthesis[1,:], '.', label='Interpolated Points')
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("s[m]")
        plt.ylabel("y[m]")
        plt.legend()
    
    def print_shape(self):
        print(self.data_synthesis.shape)
        print(self.interp_trajectory_lengths.shape)
        print(self.yaw_angles.shape)
        print(self.interpolated_x.shape)
        print(self.interpolated_y.shape)
        print(self.waypoints_x.shape)
        print(self.waypoints_y.shape)         
        


if __name__ == "__main__":
    # test_spline2d()
    path_dir = "/home/zhicun/code/atr_rl/Code/python_tools/map/config/nodes_tuve.json"
    atrPath = ATRPath(path_dir=path_dir, if_render_text=False, path_type='straight')
    # print(atrPath.interp_trajectory_lengths)
    # print(atrPath.yaw_angles)
    # print(atrPath.data_synthesis[])
    # atrPath.render()
    # print(atrPath.interpolated_x.shape)
    # print(atrPath.interp_trajectory_lengths.shape)
    # print(atrPath.yaw_angles.shape)
    distances = np.sqrt(np.sum(np.diff(atrPath.data_synthesis[0:1,:], axis=1)**2, axis=0))
    print(distances)
    # print(atrPath.data_synthesis.shape)
    plt.show()
    
    
    