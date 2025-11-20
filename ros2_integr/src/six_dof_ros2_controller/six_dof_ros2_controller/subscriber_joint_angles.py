import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from custom_interfaces.msg import JointAngles
from six_dof_ros2_controller.ros_init import init_ros_domain_from_args

SUBSCRIPTION_TOPIC = "joint_angles_topic"


class JointAngleSubscriber(Node):

    def __init__(self):
        super().__init__("joint_angle_subscriber")
        self.subscription = self.create_subscription(JointAngles, SUBSCRIPTION_TOPIC, self.listener_callback, 10)
        self.get_logger().info(f"Subscribed to {SUBSCRIPTION_TOPIC}, ready...")

    def listener_callback(self, msg):
        print("\033c", end="")  ## os.system("clear") causes the ^C to not work
        joint_angles_dict = {
            "theta1": msg.theta1,
            "theta2": msg.theta2,
            "theta3": msg.theta3,
            "theta4": msg.theta4,
            "theta5": msg.theta5,
            "theta6": msg.theta6,
        }  # parse into dict for later use

        out_buff = ""
        for k, v in joint_angles_dict.items():
            out_buff += f"{k}:{v}\r\n"
        print(out_buff)
        sys.stdout.flush()


def main(args=None):
    init_ros_domain_from_args(args)
    joint_angle_subscriber = None
    try:
        rclpy.init(args=args)
        joint_angle_subscriber = JointAngleSubscriber()
        rclpy.spin(joint_angle_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if joint_angle_subscriber is not None:
            joint_angle_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
