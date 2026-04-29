import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import serial  

class LeverPoseNode(Node):
    def __init__(self):
        super().__init__('lever_pose_detector')
        
        # مسار الموديل الخاص بك
        model_path = '/home/eng-abdullah/ros2_ws/src/lever_vision_model/runs/pose/lever_v2_model/weights/best.pt'
        self.model = YOLO(model_path) 
        self.bridge = CvBridge()
        self.latest_depth_frame = None

        self.cx, self.cy = 320, 240
        self.fx, self.fy = 554, 554

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
            self.get_logger().info('Serial Bridge: ESP32 Connected.')
        except:
            self.ser = None
            self.get_logger().warn('Serial Bridge: ESP32 not found. Continuing in Offline Mode.')

        # إعدادات الفلتر EMA
        self.smoothed_x = None
        self.smoothed_y = None
        self.smoothed_z = None
        self.smoothed_angle = None
        
        self.alpha_pos = 0.15 
        self.alpha_ang = 0.10 
        self.angle_deadband = 1.5 

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, '/lever_pose_arm', 10)

        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)

    def depth_callback(self, msg):
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        if self.latest_depth_frame is None: return
        color_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        try:
            results = self.model(color_frame, conf=0.4, verbose=False)

            for r in results:
                if r.keypoints is None or len(r.keypoints.xy) == 0 or len(r.keypoints.xy[0]) < 2:
                    continue

                try:
                    # استخراج نقاط القاعدة والطرف من YOLO
                    pivot = r.keypoints.xy[0][0]
                    tip = r.keypoints.xy[0][1]
                    u_p, v_p = int(pivot[0]), int(pivot[1])
                    u_t, v_t = int(tip[0]), int(tip[1])
                    
                    raw_angle = np.degrees(np.arctan2(v_t - v_p, u_t - u_p))

                    depth_window = self.latest_depth_frame[max(0,v_p-2):v_p+2, max(0,u_p-2):u_p+2]
                    valid_depths = depth_window[depth_window > 0]
                    
                    if len(valid_depths) > 0:
                        depth_m = float(np.median(valid_depths) - 49) / 1000.0
                        
                        msg_out = PoseStamped()
                        msg_out.header.stamp = self.get_clock().now().to_msg()
                        msg_out.header.frame_id = 'camera_link'
                        
                        raw_x = depth_m
                        raw_y = (320 - u_p) * (depth_m / 554)
                        raw_z = (240 - v_p) * (depth_m / 554)

                        # تطبيق الفلترة
                        if self.smoothed_x is None:
                            self.smoothed_x, self.smoothed_y, self.smoothed_z = raw_x, raw_y, raw_z
                            self.smoothed_angle = raw_angle
                        else:
                            self.smoothed_x += self.alpha_pos * (raw_x - self.smoothed_x)
                            self.smoothed_y += self.alpha_pos * (raw_y - self.smoothed_y)
                            self.smoothed_z += self.alpha_pos * (raw_z - self.smoothed_z)

                            if abs(raw_angle - self.smoothed_angle) > self.angle_deadband:
                                self.smoothed_angle += self.alpha_ang * (raw_angle - self.smoothed_angle)

                        msg_out.pose.position.x = self.smoothed_x
                        msg_out.pose.position.y = self.smoothed_y
                        msg_out.pose.position.z = self.smoothed_z
                        
                        # تصفير الزوايا لضمان دقة التحويل المكاني
                        msg_out.pose.orientation.x = 0.0
                        msg_out.pose.orientation.y = 0.0
                        msg_out.pose.orientation.z = 0.0
                        msg_out.pose.orientation.w = 1.0

                        try:
                            transform = self.tf_buffer.lookup_transform('arm_base_link', 'camera_link', rclpy.time.Time())
                            arm_pose = do_transform_pose(msg_out.pose, transform)
                            msg_out.pose = arm_pose
                            msg_out.header.frame_id = 'arm_base_link'
                            
                            # تحديد الاتجاه (Flag) بناءً على إحداثيات الطرف بالنسبة للقاعدة
                            if u_t > u_p:
                                msg_out.pose.orientation.x = 1.0  # الاتجاه يمين
                                dir_text = "DIRECTION: RIGHT"
                                dir_color = (0, 255, 0) # أخضر
                            else:
                                msg_out.pose.orientation.x = -1.0 # الاتجاه يسار
                                dir_text = "DIRECTION: LEFT"
                                dir_color = (0, 0, 255) # أحمر
                            
                            if self.ser:
                                payload = f"<{msg_out.pose.position.x:.3f},{msg_out.pose.position.y:.3f},{msg_out.pose.position.z:.3f},{self.smoothed_angle:.1f}>\n"
                                self.ser.write(payload.encode())

                            # إرسال البيانات للمنسق
                            self.pose_pub.publish(msg_out)

                            # الرسم على الشاشة للتأكد البصري
                            cv2.line(color_frame, (u_p, v_p), (u_t, v_t), (255, 0, 0), 2)
                            cv2.circle(color_frame, (u_t, v_t), 5, (0, 255, 255), -1) # تمييز الطرف بدائرة صفراء
                            
                            cv2.putText(color_frame, dir_text, (u_p - 20, v_p - 50), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, dir_color, 2)
                            
                            cv2.putText(color_frame, f"Ang: {self.smoothed_angle:.1f} deg", (u_p + 10, v_p - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                            
                            cv2.putText(color_frame, f"Dist: {int(self.smoothed_x*1000)} mm", (u_p + 10, v_p - 30), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        except Exception:
                            pass 

                except Exception as math_err:
                    self.get_logger().error(f"Math Error: {math_err}")

        except Exception as yolo_err:
            self.get_logger().error(f"YOLO Error: {yolo_err}")
        
        cv2.imshow("Detection (Pose Mode)", color_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LeverPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()