import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from atr_state_msgs.msg import ATRJointCommand



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(ATRJointCommand, '/joint_command_1', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ATRJointCommand()
        msg.wheel_velocity = [1.0, 2.0] # [wr, wl]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.wheel_velocity)
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