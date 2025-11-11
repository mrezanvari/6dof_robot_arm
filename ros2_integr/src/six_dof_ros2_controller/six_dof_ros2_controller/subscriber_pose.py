import os
import sys

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

# ROS_DOMAIN_ID = 123
# os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)

SUBSCRIPTION_TOPIC = "pose_topic"


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(Pose, SUBSCRIPTION_TOPIC, self.listener_callback, 10)
        self.get_logger().info(f"Subscribed to {SUBSCRIPTION_TOPIC}, ready...")

    def listener_callback(self, msg):
        print("\033c", end="")  ## os.system("clear") causes the ^C to not work

        pose_dict = {
            "position": {
                "x": msg.position.x,
                "y": msg.position.y,
                "z": msg.position.z,
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
        }

        q = (
            pose_dict["orientation"]["x"],
            pose_dict["orientation"]["y"],
            pose_dict["orientation"]["z"],
            pose_dict["orientation"]["w"],
        )

        phi, theta, psi = euler_from_quaternion(q)
        orientation_dict = {"phi": phi, "theta": theta, "psi": psi}

        out_buff = ""
        for k, v in pose_dict["position"].items():
            out_buff += f"{k}:{v}\r\n"

        for k, v in orientation_dict.items():
            out_buff += f"{k}:{v}\r\n"
        print(out_buff)
        sys.stdout.flush()


def main(args=None):
    pose_subscriber = None
    try:
        rclpy.init(args=args)
        pose_subscriber = PoseSubscriber()
        rclpy.spin(pose_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if pose_subscriber is not None:
            pose_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
