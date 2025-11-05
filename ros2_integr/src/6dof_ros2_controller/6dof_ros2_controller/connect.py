import serial
import time
import serial.tools.list_ports as ports
import subprocess


def get_esp_device():
    esp_port_manufacture = "Espressif"
    docker_port = "/dev/ttyDocker"

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

    esp32_port = esp32_connection[0].device.replace(
        "cu", "tty"
    )  # only tty works on mac but if cu works then remove this part
    print("Found", esp32_port)
    return esp32_port


def main():
    esp32_port = get_esp_device()
    subprocess.run(["sudo", "chmod", "777", esp32_port])
    ser = serial.Serial(esp32_port, 115200)
    print(ser.name)
    ser.write(b"set led 0 0 0\r\n")
    time.sleep(1.0)
    for r in range(1, 150):
        while not ser.writable:
            pass
        ser.write(f"set led {r} 0 0\r\n".encode())
        print("wrote r:", r)
        time.sleep(0.01)

    for g in range(1, 150):
        while not ser.writable:
            pass
        ser.write(f"set led {r} {g} 0\r\n".encode())
        print("wrote g:", g)
        time.sleep(0.01)

    for b in range(1, 150):
        while not ser.writable:
            pass
        ser.write(f"set led {r} {g} {b}\r\n".encode())
        print("wrote b:", b)
        time.sleep(0.01)

    ser.close()


if __name__ == "__main__":
    main()
