import re

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from custom_interfaces.msg import JointAngles
from custom_interfaces.srv import SetPose
from six_dof_ros2_controller.serial_utils import ThreadedSerial, init_esp_serial
from six_dof_ros2_controller.ros_init import init_ros_domain_from_args
from math import degrees
import time

JOINT_ANGLES_PUBLISH_TOPIC = "joint_angles_topic"
POS_PUBLISH_TOPIC = "pose_topic"
PUBLISHER_FREQ = 0.01
BAUDRATE = 2000000  # somewhat irrelevant since ESP CDC UART uses max USB1.1 speed

pattern_xyz = r"x:(-?\d+\.\d+)\s+y:(-?\d+\.\d+)\s+z:(-?\d+\.\d+)"
pattern_joint_thetas = (
    r"t1:(-?\d+\.\d+)\s+"
    r"t2:(-?\d+\.\d+)\s+"
    r"t3:(-?\d+\.\d+)\s+"
    r"t4:(-?\d+\.\d+)\s+"
    r"t5:(-?\d+\.\d+)\s+"
    r"t6:(-?\d+\.\d+)"
)
pattern_orientation = r"phi:(-?\d+\.\d+)\s+theta:(-?\d+\.\d+)\s+psi:(-?\d+\.\d+)\s+"


class SetPoseServer(Node):

    def __init__(self, serial_protocol: ThreadedSerial):
        super().__init__("set_pose_server")
        self._srv = self.create_service(SetPose, "set_pose_server", self._server_callback)
        self.serial_protocol = serial_protocol.entry_point
        self.get_logger().info("Server Ready...")

    def _server_callback(self, req, resp):
        pose = req.pose
        self.get_logger().info(
            f"Request {req.id} Position: x={pose.position.x:,.2f}, y={pose.position.y:,.2f}, z={pose.position.z:,.2f}"
        )
        # self.get_logger().info(
        #     f"Orientation: x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w}"
        # )

        orientation_quaternion = pose.orientation
        orientation_euler = euler_from_quaternion(
            [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w],
            "rzyz",
        )

        msg = f"set pos {pose.position.x} {pose.position.y} {pose.position.z} {degrees(orientation_euler[0])} {degrees(orientation_euler[1])} {degrees(orientation_euler[2])}\r\n"
        self.serial_protocol.send(msg)

        resp.success = True
        resp.id = req.id
        resp.msg = f"x={pose.position.x:,.2f}, y={pose.position.y:,.2f}, z={pose.position.z:,.2f}"
        return resp


class SetTraceServer(Node):

    def __init__(self, serial_protocol: ThreadedSerial):
        super().__init__("set_trace_server")
        self._srv = self.create_service(SetPose, "set_trace_server", self._server_callback)
        self.serial_protocol = serial_protocol.entry_point
        self.get_logger().info("Server Ready...")

    def _server_callback(self, req, resp):
        pose = req.pose

        orientation_quaternion = pose.orientation
        orientation_euler = euler_from_quaternion(
            [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w],
            "rxyz",
        )

        log_msg = f"{pose.position.x:,.2f} {pose.position.y:,.2f} {pose.position.z:,.2f} {orientation_euler[0]:,.2f} {orientation_euler[1]:,.2f} {orientation_euler[2]:,.2f}"

        self.get_logger().info(f"Request {req.id} trace: {log_msg}")

        msg = f"set trace {pose.position.x} {pose.position.y} {pose.position.z} {orientation_euler[0]} {orientation_euler[1]} {orientation_euler[2]}\r\n"
        self.serial_protocol.send(msg)

        resp.success = True
        resp.id = req.id
        resp.msg = log_msg
        return resp


class JointAnglePublisher(Node):

    def __init__(self, serial_connection: ThreadedSerial, publishing_freq: int):
        super().__init__("joint_angles_publisher")
        self.publisher_ = self.create_publisher(JointAngles, JOINT_ANGLES_PUBLISH_TOPIC, 10)
        self.serial_protocol = serial_connection.entry_point
        self.timer = self.create_timer(publishing_freq, self._timer_callback)

    def _timer_callback(self):
        msg = JointAngles()
        new_line = self.serial_protocol.currnet_line
        t_match = re.search(pattern_joint_thetas, new_line)

        if t_match:
            thetas = {f"t{i+1}": float(val) for i, val in enumerate(t_match.groups())}

            msg.theta1 = thetas["t1"]
            msg.theta2 = thetas["t2"]
            msg.theta3 = thetas["t3"]
            msg.theta4 = thetas["t4"]
            msg.theta5 = thetas["t5"]
            msg.theta6 = thetas["t6"]

            self.publisher_.publish(msg)


class PosePublisher(Node):

    def __init__(self, serial_connection: ThreadedSerial, publishing_freq: int):
        super().__init__("pose_publisher")
        self.publisher_ = self.create_publisher(Pose, POS_PUBLISH_TOPIC, 10)
        self.serial_protocol = serial_connection.entry_point
        self.timer = self.create_timer(publishing_freq, self._timer_callback)

    def _timer_callback(self):
        msg = Pose()
        new_line = self.serial_protocol.currnet_line
        xyz_match = re.search(pattern_xyz, new_line)
        orientation_match = re.search(pattern_orientation, new_line)

        if xyz_match and orientation_match:

            msg.position.x = float(xyz_match.group(1))
            msg.position.y = float(xyz_match.group(2))
            msg.position.z = float(xyz_match.group(3))

            orientation_quaternion = quaternion_from_euler(
                float(orientation_match.group(1)),
                float(orientation_match.group(2)),
                float(orientation_match.group(3)),
                "rzyz",
            )

            msg.orientation.x = orientation_quaternion[0]
            msg.orientation.y = orientation_quaternion[1]
            msg.orientation.z = orientation_quaternion[2]
            msg.orientation.w = orientation_quaternion[3]

            self.publisher_.publish(msg)


def init_robot(serial_conn: ThreadedSerial):
    serial_conn.entry_point.send("set gain 6\r\n")
    time.sleep(1)
    serial_conn.entry_point.send("set interval 0\r\n")
    time.sleep(1)
    serial_conn.entry_point.send("jacobi\r\n")
    time.sleep(1)


def main(args=None):

    init_ros_domain_from_args(args)

    server_pose_node = None
    server_trace_node = None
    joint_angle_publisher_node = None
    pose_publisher_node = None
    try:
        serial_connection_thread = init_esp_serial(BAUDRATE, True)
        init_robot(serial_connection_thread)

        rclpy.init(args=args)

        server_pose_node = SetPoseServer(serial_connection_thread)
        server_trace_node = SetTraceServer(serial_connection_thread)
        joint_angle_publisher_node = JointAnglePublisher(serial_connection_thread, PUBLISHER_FREQ)
        pose_publisher_node = PosePublisher(serial_connection_thread, PUBLISHER_FREQ)

        executor = MultiThreadedExecutor()
        executor.add_node(server_pose_node)
        executor.add_node(server_trace_node)
        executor.add_node(joint_angle_publisher_node)
        executor.add_node(pose_publisher_node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server_pose_node is not None:
            server_pose_node.destroy_node()
        if server_trace_node is not None:
            server_trace_node.destroy_node()
        if joint_angle_publisher_node is not None:
            joint_angle_publisher_node.destroy_node()
        if pose_publisher_node is not None:
            pose_publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
