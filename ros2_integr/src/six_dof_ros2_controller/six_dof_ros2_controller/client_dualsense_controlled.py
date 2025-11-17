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
from six_dof_ros2_controller.DualSenseManager import DualSenseActionLayer
from pynput import keyboard

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


class SetPoseDSClient(Node):

    def __init__(self):
        super().__init__("set_trace_ds_client")
        self._client_trace = self.create_client(SetPose, "set_trace_server")
        self._client_pose = self.create_client(SetPose, "set_pose_server")

        while not self._client_trace.wait_for_service(timeout_sec=1.0) or not self._client_pose.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for 'set_trace_server' and 'set_pose_server' servers...")

        self.ds_controller = DualSenseActionLayer()
        self.ds_controller.lightbar.set_color(0, 50, 250)
        self._timer = self.create_timer(UPDATE_FREQ_S, self._timer_callback)

    def _timer_callback(self):

        if not self.ds_controller.poll():
            return

        req = SetPose.Request()
        req.id = random.randint(0, 5000)
        req.pose = Pose()

        if self.ds_controller.current_joysticks.left.btn:
            req.pose = home_pose
            self.future = self._client_pose.call_async(req)
            self.future.add_done_callback(self._response_callback)
            return

        req.pose.position.x = self.ds_controller.current_joysticks.left.x
        req.pose.position.z = self.ds_controller.current_joysticks.left.y
        y_change = 0.0
        if self.ds_controller.btn_up.pressed:
            y_change = self.ds_controller.dpad_values[0]
        elif self.ds_controller.btn_down.pressed:
            y_change = -self.ds_controller.dpad_values[1]
        req.pose.position.y = y_change

        orientation_phi = self.ds_controller.current_joysticks.right.x
        orientation_theta = self.ds_controller.current_joysticks.right.y
        orientation_psi = 0.0
        if self.ds_controller.btn_right.pressed:
            orientation_psi = self.ds_controller.dpad_values[3]
        elif self.ds_controller.btn_left.pressed:
            orientation_psi = -self.ds_controller.dpad_values[2]

        q_of_trace = quaternion_from_euler(orientation_phi, orientation_theta, orientation_psi, "rxyz")
        req.pose.orientation.x = q_of_trace[0]
        req.pose.orientation.y = q_of_trace[1]
        req.pose.orientation.z = q_of_trace[2]
        req.pose.orientation.w = q_of_trace[3]

        self.get_logger().info(
            f"Sending trace: {req.pose.position.x:,.2f}, {req.pose.position.y:,.2f}, {req.pose.position.z:,.2f} {req.pose.orientation}"
        )

        self.future = self._client_trace.call_async(req)
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
        client_node = SetPoseDSClient()
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
