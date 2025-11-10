import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from custom_interfaces.srv import SetPose
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor
import serial
import time
import serial.tools.list_ports as ports
import subprocess
import os
import re

# ROS_DOMAIN_ID = 123
# os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)

pattern_xyz = r"x:(-?\d+\.\d+)\s+y:(-?\d+\.\d+)\s+z:(-?\d+\.\d+)"
pattern_t = (
    r"t1:(-?\d+\.\d+)\s+"
    r"t2:(-?\d+\.\d+)\s+"
    r"t3:(-?\d+\.\d+)\s+"
    r"t4:(-?\d+\.\d+)\s+"
    r"t5:(-?\d+\.\d+)\s+"
    r"t6:(-?\d+\.\d+)"
)


def get_esp_device() -> serial.Serial:
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


def init_esp_serial():
    esp32_port = get_esp_device()
    subprocess.run(["sudo", "chmod", "777", esp32_port])
    esp_serial_connection = serial.Serial(esp32_port, 115200)
    print(esp_serial_connection.name)
    esp_serial_connection.write(b"set led 0 0 0\r\n")
    time.sleep(1.0)
    return esp_serial_connection


class MinimalPublisher(Node):

    def __init__(self, serial_connection: serial.Serial, publishing_freq: int):
        super().__init__("joint_angles_publisher")
        self.publisher_ = self.create_publisher(String, "joint_angles_topic", 10)
        self.serial_connection = serial_connection
        self.timer = self.create_timer(publishing_freq, self._timer_callback)

    def _timer_callback(self):
        msg = String()
        new_line = self.serial_connection.readline().decode()
        xyz_match = re.search(pattern_xyz, new_line)
        t_match = re.search(pattern_t, new_line)

        if xyz_match and t_match:
            xyz = {
                "x": float(xyz_match.group(1)),
                "y": float(xyz_match.group(2)),
                "z": float(xyz_match.group(3)),
            }  # parse into dict for future use
            thetas = {f"t{i+1}": float(val) for i, val in enumerate(t_match.groups())}

            msg.data = f"x:{xyz['x']}\r\ny:{xyz['y']}\r\nz:{xyz['z']}\r\n"  # for now just print normally
            for k, v in thetas.items():
                msg.data += f"{k}:{v}\r\n"
            self.publisher_.publish(msg)


class MinimalServer(Node):

    def __init__(self, serial_connection: serial.Serial):
        super().__init__("set_pose_server")
        self._srv = self.create_service(SetPose, "set_pose_server", self._server_callback)
        self.serial_connection = serial_connection
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
        self.serial_connection.write(
            f"set pos {pose.position.x} {pose.position.y} {pose.position.z} {orientation_euler[0]} {orientation_euler[1]} {orientation_euler[2]}\r\n".encode()
        )

        resp.success = True
        resp.id = req.id
        resp.msg = f"x={pose.position.x:,.2f}, y={pose.position.y:,.2f}, z={pose.position.z:,.2f}"
        return resp


def main(args=None):
    server_node = None
    joint_angle_publisher_node = None
    try:
        ser_conn = init_esp_serial()
        rclpy.init(args=args)

        server_node = MinimalServer(ser_conn)
        joint_angle_publisher_node = MinimalPublisher(ser_conn, 0.01)

        executor = MultiThreadedExecutor()
        executor.add_node(server_node)
        executor.add_node(joint_angle_publisher_node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server_node is not None:
            server_node.destroy_node()
        if joint_angle_publisher_node is not None:
            joint_angle_publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
