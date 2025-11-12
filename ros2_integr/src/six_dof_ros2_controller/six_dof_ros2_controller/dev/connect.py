import time
from six_dof_ros2_controller.serial_utils import init_esp_serial


def main():
    ser = init_esp_serial(115200)
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
