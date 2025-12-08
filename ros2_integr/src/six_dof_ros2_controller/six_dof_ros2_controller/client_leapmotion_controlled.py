import os
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from math import degrees, radians

from custom_interfaces.srv import SetPose
from geometry_msgs.msg import Pose, Quaternion
from tf_transformations import quaternion_from_euler
from six_dof_ros2_controller.ros_init import init_ros_domain_from_args
import socket
import re
from six_dof_ros2_controller.server import pattern_orientation, pattern_xyz

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

UPDATE_FREQ_S = 0.01

home_pose = Pose()
home_pose.position.x = 410.0
home_pose.position.y = 215.0
home_pose.position.z = 0.0
pose_orientation_quaternion = quaternion_from_euler(radians(90), radians(90), 0, "rzyz")
home_pose.orientation.x = pose_orientation_quaternion[0]
home_pose.orientation.y = pose_orientation_quaternion[1]
home_pose.orientation.z = pose_orientation_quaternion[2]
home_pose.orientation.w = pose_orientation_quaternion[3]


class SetPoseLeapMotionClient(Node):

    def __init__(self):
        super().__init__("set_pose_leap_client")
        self._client = self.create_client(SetPose, "set_pose_server")

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'set_trace_server' and 'set_pose_server' servers...")

        self._timer = self.create_timer(UPDATE_FREQ_S, self._timer_callback)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))

    def _timer_callback(self):

        data, addr = self.sock.recvfrom(1024)

        if data == "" or data is None:
            return

        req = SetPose.Request()
        req.id = random.randint(0, 5000)
        req.pose = Pose()

        new_line = data.decode()
        xyz_match = re.search(pattern_xyz, new_line)
        orientation_match = re.search(pattern_orientation, new_line)

        if xyz_match and orientation_match:

            req.pose.position.x = float(xyz_match.group(1))
            req.pose.position.y = float(xyz_match.group(2))
            req.pose.position.z = float(xyz_match.group(3))

            orientation_quaternion = quaternion_from_euler(
                float(orientation_match.group(1)),
                float(orientation_match.group(2)),
                float(orientation_match.group(3)),
                "rzyz",
            )

            req.pose.orientation.x = orientation_quaternion[0]
            req.pose.orientation.y = orientation_quaternion[1]
            req.pose.orientation.z = orientation_quaternion[2]
            req.pose.orientation.w = orientation_quaternion[3]

            self.get_logger().info(
                f"Sending pose from {addr}: {req.pose.position.x:,.2f}, {req.pose.position.y:,.2f}, {req.pose.position.z:,.2f} {req.pose.orientation}"
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
    init_ros_domain_from_args(args)
    client_node = None
    try:
        rclpy.init(args=args)
        client_node = SetPoseLeapMotionClient()
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
