import numpy as np
import matplotlib.pyplot as plt

class ATR_RK4():
    
    def __init__(self,
                 params,
                 init_state = np.array([0, 0, 0], dtype=float),):
        self.wheel_radius = params.wheel_radius
        self.track_width = params.track_width
        self.state = init_state
        self.dt = params.dt
    
    def simulate_slip(self, wheel_speeds, slip_probability=0.1, max_speed=2):
        """
        Simulate wheel slipping based on a given probability.
        
        :param wheel_speeds: Array of wheel speeds.
        :param slip_probability: Probability of slipping occurring.
        :param max_speed: Maximum allowed speed for the wheels.
        :return: Adjusted wheel speeds.
        """
        # Apply slip based on the slip probability
        if np.random.rand() < slip_probability:
            # Simulate slip by setting wheel speeds to zero
            wheel_speeds = np.array([0, 0])
        else:
            # No slip, optionally add some Gaussian noise
            wheel_speeds += np.random.normal(-0.2, 0.1, size=2)

        # Ensure wheel speeds are within the allowed range
        return np.clip(wheel_speeds, -max_speed, max_speed)
        
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

    def runge_kutta_4_step(self, wheel_speeds:list, method:str="RK4"):
        """
        simulate the robot's movement
        
        ### Parameters
        - `wheel_speeds`: [wr, wl].
        - `method`: "RK4" or "simple"(only use k1)
        
        ### Returns
        - `state`: [x, y, theta]
        - `linear_velocity`: linear velocity m/s
        - `angular_velocity`: angular velocity rad/s
        
        ### Example
        >>> state, linear_velocity, angular_velocity = runge_kutta_4_step([0.1, 0.1])
        """
        
        # switch wheel speed sequence
        wheel_speeds = [wheel_speeds[1], wheel_speeds[0]] # now wheel_speed is [wl, wr]
        # Add noise to the wheel speeds
        # wheel_speeds = np.clip(wheel_speeds + np.random.normal(-0.2, 0.1, size=2), -2, 2)
        # left wheel with positive noise, right wheel with negative noise
        # add random slip on the wheels
        # self.simulate_slip(wheel_speeds)
        left_wheel_speed, right_wheel_speed = wheel_speeds
        self.linear_velocity = self.wheel_radius * (left_wheel_speed + right_wheel_speed) / 2
        self.angular_velocity = self.wheel_radius * \
            (right_wheel_speed - left_wheel_speed) / self.track_width
        if method == "simple":
            k1 = self.robot_pose_derivative(self.state, wheel_speeds)
            self.state = self.state + self.dt * k1
            self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
            return np.copy(self.state), self.linear_velocity, self.angular_velocity
        else:  
            k1 = self.robot_pose_derivative(self.state, wheel_speeds)
            k2 = self.robot_pose_derivative(self.state + self.dt / 2 * k1, wheel_speeds)
            k3 = self.robot_pose_derivative(self.state + self.dt / 2 * k2, wheel_speeds)
            k4 = self.robot_pose_derivative(self.state + self.dt * k3, wheel_speeds)

            self.state = self.state + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            x_normalized = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
            self.state[2] = x_normalized
            return np.copy(self.state), self.linear_velocity, self.angular_velocity
    
    def reset(self, position:np.ndarray):
        self.state = position
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0