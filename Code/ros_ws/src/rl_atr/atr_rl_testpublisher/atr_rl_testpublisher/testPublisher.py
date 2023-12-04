import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from atr_state_msgs.msg import ATRJointCommand

import gymnasium as gym
import torch
import torch.nn as nn

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(ATRJointCommand, '/joint_command_1', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ATRJointCommand()
        random_values = 0.0 * torch.rand(2)
        random_values_list = random_values.tolist()
        msg.wheel_velocity = random_values_list # [wr, wl]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.wheel_velocity)
        self.get_logger().info('TESTINGG')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()