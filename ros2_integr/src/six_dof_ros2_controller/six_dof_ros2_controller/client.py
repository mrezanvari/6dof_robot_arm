import os
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from math import degrees, radians

from custom_interfaces.srv import SetPose
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
from six_dof_ros2_controller.ros_init import init_ros_domain_from_args

UPDATE_FREQ_S = 8


base_pose = Pose()
base_pose.position.x = 410.0
base_pose.position.y = 215.0
base_pose.position.z = 0.0
base_orientation_quaternion = quaternion_from_euler(radians(90), radians(60), 0, "rzyz")
base_pose.orientation.x = base_orientation_quaternion[0]
base_pose.orientation.y = base_orientation_quaternion[1]
base_pose.orientation.z = base_orientation_quaternion[2]
base_pose.orientation.w = base_orientation_quaternion[3]

other_pose = Pose()
other_pose.position.x = 410.0
other_pose.position.y = 215.0
other_pose.position.z = 0.0
base_orientation_quaternion = quaternion_from_euler(radians(90), radians(170), 0, "rzyz")
other_pose.orientation.x = base_orientation_quaternion[0]
other_pose.orientation.y = base_orientation_quaternion[1]
other_pose.orientation.z = base_orientation_quaternion[2]
other_pose.orientation.w = base_orientation_quaternion[3]


pose_list = [
    base_pose,
    other_pose,
]

pose_indx = 0


def increment_pose():
    global pose_indx
    out_pose = pose_list[pose_indx]
    pose_indx = pose_indx + 1 if (len(pose_list) - 1) > pose_indx else 0
    return out_pose


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
        req.pose = increment_pose()

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
    init_ros_domain_from_args(args)
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
