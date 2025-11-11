import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from custom_interfaces.srv import SetPose
from custom_interfaces.msg import JointAngles
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.executors import MultiThreadedExecutor
from serial import Serial
from serial.threaded import LineReader
import time
import serial.tools.list_ports as ports
from serial.threaded import LineReader, ReaderThread
from queue import Queue
import subprocess
import os
import re

# ROS_DOMAIN_ID = 123
# os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)

JOINT_ANGLES_PUBLISH_TOPIC = "joint_angles_topic"
POS_PUBLISH_TOPIC = "pose_topic"
PUBLISHER_FREQ = 0.01

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


def get_esp_device() -> Serial:
    esp_port_manufacture = "Espressif"
    docker_port = "/dev/ttyDocker"
    if os.path.exists(docker_port):
        print("Found", docker_port)
        return docker_port

    serial_ports = list(ports.comports())
    esp32_connection = [i for i in serial_ports if i.manufacturer == esp_port_manufacture]

    if len(esp32_connection) == 0:
        print("ERROR: ESP32 not connected!")
        exit()

    esp32_port = esp32_connection[0].device.replace(
        "cu", "tty"
    )  # only tty works on mac but if cu works then remove this part
    print("Found", esp32_port)
    return esp32_port


class LineReaderProtocol(LineReader):
    TERMINATOR = b"\r\n"

    def __init__(self):
        super().__init__()
        self.currnet_line = ""

    def connection_made(self, transport):
        print(f"New serial protocol: {transport.name}")
        return super().connection_made(transport)

    def handle_line(self, line):
        self.currnet_line = line

    def send(self, msg: str):
        self.transport.write((msg + "\r\n").encode())


class ThreadedSerial:

    def __init__(self, port, baudrate):
        self.ser = Serial(port, baudrate)
        self.thread = ReaderThread(self.ser, LineReaderProtocol)
        self.thread.start()
        connection = self.thread.connect()  # just to be sure connection is made
        self.entry_point = connection[1]  # holds the line protocol class

    def stop(self):
        self.thread.close()


def init_esp_serial() -> ThreadedSerial:
    esp32_port = get_esp_device()
    subprocess.run(["sudo", "chmod", "777", esp32_port])
    esp_serial_connection = ThreadedSerial(esp32_port, 115200)
    print(esp32_port)
    time.sleep(1.0)
    return esp_serial_connection


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
            )

            msg.orientation.x = orientation_quaternion[0]
            msg.orientation.y = orientation_quaternion[1]
            msg.orientation.z = orientation_quaternion[2]
            msg.orientation.w = orientation_quaternion[3]

            self.publisher_.publish(msg)


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
            [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
        )

        msg = f"set pos {pose.position.x} {pose.position.y} {pose.position.z} {orientation_euler[0]} {orientation_euler[1]} {orientation_euler[2]}\r\n"
        self.serial_protocol.send(msg)

        resp.success = True
        resp.id = req.id
        resp.msg = f"x={pose.position.x:,.2f}, y={pose.position.y:,.2f}, z={pose.position.z:,.2f}"
        return resp


def main(args=None):
    server_node = None
    joint_angle_publisher_node = None
    pose_publisher_node = None
    try:
        serial_connection_thread = init_esp_serial()
        rclpy.init(args=args)

        server_node = SetPoseServer(serial_connection_thread)
        joint_angle_publisher_node = JointAnglePublisher(serial_connection_thread, PUBLISHER_FREQ)
        pose_publisher_node = PosePublisher(serial_connection_thread, PUBLISHER_FREQ)

        executor = MultiThreadedExecutor()
        executor.add_node(server_node)
        executor.add_node(joint_angle_publisher_node)
        executor.add_node(pose_publisher_node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server_node is not None:
            server_node.destroy_node()
        if joint_angle_publisher_node is not None:
            joint_angle_publisher_node.destroy_node()
        if pose_publisher_node is not None:
            pose_publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
