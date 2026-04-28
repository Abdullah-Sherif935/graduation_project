import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdVelToWheel(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wheel')

        # parameters: أقصى power و المسافة بين العجلتين L (متر)
        self.declare_parameter('max_power', 250.0)   # نفس مدى أوامر ESP
        self.declare_parameter('wheel_base', 0.5)    # عدّلها بعدين حسب عرض الروبوت الحقيقي

        self.max_power = float(self.get_parameter('max_power').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        # Subscriber على /cmd_vel (Twist)
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher على /wheel_cmd (String: "R L")
        self.pub = self.create_publisher(
            String,
            '/wheel_cmd',
            10
        )

        self.get_logger().info(
            f"cmd_vel_to_wheel started. max_power={self.max_power}, wheel_base={self.wheel_base}"
        )

    def cmd_vel_callback(self, msg: Twist):
        """
        تحويل من (v,w) إلى (R,L) ثم نشرها في شكل String "R L".
        v = linear.x (m/s)
        w = angular.z (rad/s)
        """
        v = msg.linear.x
        w = msg.angular.z

        # نموذج diff-drive:
        # v_r = v + w * L/2
        # v_l = v - w * L/2
        v_r = v + w * self.wheel_base / 2.0
        v_l = v - w * self.wheel_base / 2.0

        # نعمل scaling لتناسب max_power
        # منغيرش scale لو السرعات صغيرة (ما تتخطاش max_power)
        max_abs = max(abs(v_r), abs(v_l), 1e-6)  # نتجنب القسمة على صفر
        if max_abs > 1.0:
            # لو السرعات كبيرة، scale بحيث أكبر واحدة = 1.0
            v_r /= max_abs
            v_l /= max_abs

        # نحولها لمجال -max_power .. +max_power
        p_r = v_r * self.max_power
        p_l = v_l * self.max_power

        # saturation + تحويل ل int
        p_r = int(max(-self.max_power, min(self.max_power, p_r)))
        p_l = int(max(-self.max_power, min(self.max_power, p_l)))

        # تجهيز رسالة "R L"
        out = String()
        out.data = f"{p_r} {p_l}"

        self.pub.publish(out)

        self.get_logger().info(
            f"cmd_vel: v={v:.2f}, w={w:.2f}  ->  wheel_cmd: {out.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
