import rclpy
import os, sys
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import String

# ROS_DOMAIN_ID = 123
# os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("joint_angle_subscriber")
        self.subscription = self.create_subscription(String, "joint_angles_topic", self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscribed to 'joint_angles_topic', ready...")

    def listener_callback(self, msg):
        os.system("clear")
        print(msg.data)
        sys.stdout.flush()


def main(args=None):
    minimal_subscriber = None
    try:
        rclpy.init(args=args)
        minimal_subscriber = MinimalSubscriber()
        rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if minimal_subscriber is not None:
            minimal_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
