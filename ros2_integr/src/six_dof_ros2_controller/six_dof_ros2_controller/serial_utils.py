import os
import subprocess
import time

import serial.tools.list_ports as ports
from serial import Serial
from serial.threaded import LineReader, ReaderThread
from typing import Union


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
        self.transport.write(msg.encode())


class ThreadedSerial:

    def __init__(self, port, baudrate):
        self.ser = Serial(port, baudrate)
        self.thread = ReaderThread(self.ser, LineReaderProtocol)
        self.thread.start()
        connection = self.thread.connect()  # just to be sure connection is made
        self.entry_point = connection[1]  # holds the line protocol class

    def stop(self):
        self.thread.close()


def init_esp_serial(baudrate: int, threaded: bool = False) -> Union[ThreadedSerial, Serial]:
    esp32_port = get_esp_device()
    subprocess.run(["sudo", "chmod", "777", esp32_port])
    initfunc = ThreadedSerial if threaded else Serial
    esp_serial_connection = initfunc(esp32_port, baudrate)
    print(esp32_port)
    time.sleep(1.0)
    return esp_serial_connection
