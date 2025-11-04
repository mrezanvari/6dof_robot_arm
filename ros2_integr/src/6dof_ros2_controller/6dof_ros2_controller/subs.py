# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports as ports
import subprocess
import re


def get_esp_device():
    esp_port_manufacture = "Espressif"
    docker_port = '/dev/ttyDocker'

    serial_ports = list(ports.comports())

    esp32_connection = [i for i in serial_ports if i.manufacturer == esp_port_manufacture]
    if len(esp32_connection) == 0:
        try:
            tempSer = serial.Serial(docker_port)
            tempSer.close()
        except:
            print("ERROR: ESP32 not connected!")
            exit()
        finally:
            esp32_port = docker_port
            print("Found", esp32_port)
            return esp32_port

    esp32_port = esp32_connection[0].device.replace("cu", "tty")  # only tty works on mac but if cu works then remove this part
    print("Found", esp32_port)
    return esp32_port

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(String, "topic", self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        esp32_port = get_esp_device()
        subprocess.run(["sudo", "chmod", "777", esp32_port])
        self.port = serial.Serial(esp32_port, 115200)
        print("Connected to:", self.port.name)
        self.port.write(b"set led 0 0 0\r\n")
        print("Waiting for message...\r\n")

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        match = re.search(r"\d+", msg.data)  # finds one or more digits
        if match:
            b_val = int(match.group())
            self.port.write(f"set led 0 0 {b_val}\r\n".encode())
            self.get_logger().info(f"Updated LED b_val to:{b_val}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
