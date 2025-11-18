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
from pynput import keyboard
from pynput.keyboard import Key
from sys import stdout

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

key_inc_curve = 0
key_inc = 0

key_states = {
    Key.up: False,  # x+
    Key.down: False,  # x-
    Key.left: False,  # z+
    Key.right: False,  # z-
    ".": False,  # y+
    "/": False,  # y-
    #
    "w": False,  # phi+
    "s": False,  # phi-
    "a": False,  # theta+
    "d": False,  # theta-
    "q": False,  # psi+
    "e": False,  # psi-
    Key.space: False,  # home
}


def key_value(key):
    return str.lower(key.char) if (hasattr(key, "char")) else key


def on_key_press(key):
    global key_inc_curve, key_inc

    if not (key_value(key) in key_states.keys()):
        return

    key_inc_curve += 0.01
    key_inc += 0.01 if key_inc_curve > 0.1 else 0.0
    key_inc = max(key_inc, 0.1)
    key_inc = min(key_inc, 0.5)
    key_states[key_value(key)] = True


def on_key_release(key):
    global key_inc_curve, key_inc
    if not (key_value(key) in key_states.keys()):
        return

    key_states[key_value(key)] = False
    if all(not v for v in iter(key_states.values())):
        key_inc_curve = 0.0
        key_inc = 0.0


class SetPoseKeyboardClient(Node):

    def __init__(self, key_listener: keyboard.Listener):
        super().__init__("set_trace_ds_client")
        self._client_trace = self.create_client(SetPose, "set_trace_server")
        self._client_pose = self.create_client(SetPose, "set_pose_server")

        while not self._client_trace.wait_for_service(timeout_sec=1.0) or not self._client_pose.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Waiting for 'set_trace_server' and 'set_pose_server' servers...")

        self.keyboard_listener = key_listener
        self.last_pose = Pose()
        self._timer = self.create_timer(UPDATE_FREQ_S, self._timer_callback)

    def _timer_callback(self):
        global key_inc, key_states

        req = SetPose.Request()
        req.id = random.randint(0, 5000)

        if key_states[Key.space]:
            if self.last_pose == home_pose:
                return
            req.pose = home_pose
            self.last_pose = home_pose
            self.future = self._client_pose.call_async(req)
            self.future.add_done_callback(self._response_callback)
            return

        temp_pose = Pose()

        temp_pose.position.x = float(key_inc * (key_states[Key.up] or (key_states[Key.down] * -1)))
        temp_pose.position.z = float(key_inc * (key_states[Key.right] or (key_states[Key.left] * -1)))
        temp_pose.position.y = float(key_inc * (key_states["."] or (key_states["/"] * -1)))

        orientation_phi = float(key_inc * (key_states["w"] or (key_states["s"] * -1)))
        orientation_theta = float(key_inc * (key_states["d"] or (key_states["a"] * -1)))
        orientation_psi = float(key_inc * (key_states["e"] or (key_states["q"] * -1)))

        q_of_trace = quaternion_from_euler(orientation_phi, orientation_theta, orientation_psi, "rxyz")
        temp_pose.orientation.x = q_of_trace[0]
        temp_pose.orientation.y = q_of_trace[1]
        temp_pose.orientation.z = q_of_trace[2]
        temp_pose.orientation.w = q_of_trace[3]

        print("\033c", end="")
        print(
            f"Trace vals: {temp_pose.position.x:,.2f}, {temp_pose.position.y:,.2f}, {temp_pose.position.z:,.2f}, {orientation_phi:,.2f}, {orientation_theta:,.2f}, {orientation_psi:,.2f}"
        )
        stdout.flush()

        if temp_pose == self.last_pose:
            return

        req.pose = self.last_pose = temp_pose

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
    global_key_listener = keyboard.Listener(on_press=on_key_press, on_release=on_key_release, suppress=False)
    client_node = None
    try:
        rclpy.init(args=args)
        global_key_listener.start()
        client_node = SetPoseKeyboardClient(global_key_listener)
        rclpy.spin(client_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        global_key_listener.stop()
        if client_node is not None:
            client_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
