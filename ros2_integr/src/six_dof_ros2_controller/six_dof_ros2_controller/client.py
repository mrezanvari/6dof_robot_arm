import os
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from custom_interfaces.srv import SetPose

# ROS_DOMAIN_ID = 123
# os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)

UPDATE_FREQ_S = 1.5


class SetPoseClient(Node):

    def __init__(self):
        super().__init__("set_pose_client")
        self._client = self.create_client(SetPose, "set_pose_server")

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'set_pose_server' server...")

        self._timer = self.create_timer(UPDATE_FREQ_S, self._timer_callback)

    def _timer_callback(self):
        req = SetPose.Request()
        req.id = random.randint(0, 5000)
        req.pose.position.x = random.uniform(-250, 250)
        req.pose.position.y = random.uniform(0, 100)
        req.pose.position.z = random.uniform(0, 250)

        req.pose.orientation.x = random.uniform(0, 180)
        req.pose.orientation.y = random.uniform(0, 180)
        req.pose.orientation.z = random.uniform(0, 180)
        req.pose.orientation.w = random.uniform(0, 250)

        self.get_logger().info(
            f"Sending Position: x={req.pose.position.x:,.2f}, y={req.pose.position.y:,.2f}, z={req.pose.position.z:,.2f}"
        )

        self.future = self._client.call_async(req)
        self.future.add_done_callback(self._response_callback)

    def _response_callback(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f"Request {resp.id} {'succeded' if resp.success else 'faild'}")
        except Exception as ex:
            self.get_logger().info(f"{str(ex)}")


def main(args=None):
    client_node = None
    try:
        rclpy.init(args=args)
        client_node = SetPoseClient()
        rclpy.spin(client_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if client_node is not None:
            client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
