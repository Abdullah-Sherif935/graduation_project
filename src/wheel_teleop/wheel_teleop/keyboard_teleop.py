import rclpy
from rclpy.node import Node
import sys
import termios
import tty
import serial
import time

class WheelTeleop(Node):
    def __init__(self):
        super().__init__('wheel_keyboard_teleop')

        # ===== Serial to ESP32 =====
        self.port = '/dev/ttyACM0'
        self.baud = 115200

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
        except Exception as e:
            self.get_logger().fatal(f"Cannot open serial {self.port}: {e}")
            raise

        # Speeds
        self.step = 10
        self.max_speed = 250
        self.r = 0
        self.l = 0

        self.get_logger().info("Wheel teleop started (DIRECT SERIAL MODE)")
        self.get_logger().info("w: forward | s: back | a: left | d: right | x: stop")

        # Keyboard setup
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def send(self):
        cmd = f"{self.r} {self.l}\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent -> R: {self.r}, L: {self.l}")

    def run(self):
        try:
            while rclpy.ok():
                key = sys.stdin.read(1)

                if key == 'w':
                    self.r += self.step
                    self.l += self.step
                elif key == 's':
                    self.r -= self.step
                    self.l -= self.step
                elif key == 'a':
                    self.r -= self.step
                    self.l += self.step
                elif key == 'd':
                    self.r += self.step
                    self.l -= self.step
                elif key == 'x':
                    self.r = 0
                    self.l = 0
                elif key == '\x03':  # CTRL+C
                    break

                # Saturation
                self.r = max(-self.max_speed, min(self.max_speed, self.r))
                self.l = max(-self.max_speed, min(self.max_speed, self.l))

                self.send()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.ser.close()

def main():
    rclpy.init()
    node = WheelTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
