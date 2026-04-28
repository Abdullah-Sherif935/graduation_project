import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time


class WheelCmdToSerial(Node):
    def __init__(self):
        super().__init__('wheel_cmd_to_serial')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Open serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial port {port}: {e}")
            raise

        # Subscriber
        self.sub = self.create_subscription(
            String,
            '/wheel_cmd',
            self.callback,
            10
        )

        self.get_logger().info(
            f"wheel_cmd_to_serial started → {port} @ {baud}"
        )

    def callback(self, msg: String):
        cmd = msg.data.strip() + '\n'
        self.ser.write(cmd.encode())
        self.get_logger().debug(f"Sent to ESP: {cmd.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelCmdToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
