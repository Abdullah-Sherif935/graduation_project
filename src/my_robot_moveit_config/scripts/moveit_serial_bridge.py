#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import serial
import math

class MoveItSerialBridge(Node):
    def __init__(self):
        super().__init__('moveit_serial_bridge')
        
        # --- إعدادات السيريال ---
        self.serial_port = '/dev/ttyACM0' 
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to ESP32 on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Serial: {e}')
            return

        # أسماء المفاصل الستة
        self.arm_joint_names = [
            'j1_arm_joint', 'j2_arm_joint', 'j3_arm_joint',
            'j4_arm_joint', 'j5_arm_joint', 'j6_arm_joint'
        ]
        
        # 🛠️ لوحة تحكم الاتجاهات (Direction Multipliers)
        # 1 = اتجاه طبيعي | -1 = اتجاه معكوس
        # الترتيب: [J1, J2, J3, J4, J5, J6]
        self.joint_directions = [-1, 1, 1, 1, 1, 1]
        
        # الاشتراك في مسار التخطيط
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            10)

    def trajectory_callback(self, msg):
        try:
            # التأكد إن في مسار مبعوت
            if not msg.trajectory:
                return

            # سحب نقاط المسار
            joint_traj = msg.trajectory[0].joint_trajectory
            names = joint_traj.joint_names
            
            # سحب "آخر نقطة" في المسار (الهدف النهائي)
            final_point = joint_traj.points[-1].positions
            
            angles_deg = [0.0] * 6
            
            # ترتيب الزوايا وتطبيق معامل الاتجاه
            for i, target_name in enumerate(self.arm_joint_names):
                if target_name in names:
                    index = names.index(target_name)
                    angle_rad = final_point[index]
                    
                    # 🔄 ضرب الزاوية في معامل الاتجاه الخاص بها من المصفوفة
                    angle_rad = angle_rad * self.joint_directions[i]
                    
                    # التحويل لدرجات وحفظها في اللستة
                    angles_deg[i] = math.degrees(angle_rad)

            # تجميع الزوايا وإرسالها
            serial_msg = "{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}\n".format(*angles_deg)
            self.ser.write(serial_msg.encode('utf-8'))
            
            # طباعة الهدف للتأكيد على الشاشة
            self.get_logger().info(f'Sent Target: {serial_msg.strip()}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing trajectory: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveItSerialBridge()
    if hasattr(node, 'ser'):
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.ser.close()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()