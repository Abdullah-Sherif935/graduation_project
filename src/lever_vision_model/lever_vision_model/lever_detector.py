import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import serial  
from collections import deque 

class LeverPoseNode(Node):
    def __init__(self):
        super().__init__('lever_pose_node')
        
        # 1. إعدادات الموديل (تأكد من المسار)
        model_path = '/home/abdullah/ros2_ws/src/lever_vision_model/runs/pose/lever_v2_model/weights/best.pt'
        self.model = YOLO(model_path) 
        self.bridge = CvBridge()
        self.latest_depth_frame = None

        # ثوابت الكاميرا Astra Pro
        self.cx, self.cy = 320, 240
        self.fx, self.fy = 554, 554

        # 2. إعداد السيريال (مغلف بـ try عشان ميقفلش الكود لو الـ ESP مش واصلة)
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
            self.get_logger().info('Serial Bridge: ESP32 Connected.')
        except:
            self.ser = None
            self.get_logger().warn('Serial Bridge: ESP32 not found. Continuing in Offline Mode.')

        # 3. الفلتر (Moving Average) لآخر 10 قراءات
        self.history = deque(maxlen=10)

        # 4. إعدادات ROS (Publisher & TF)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.point_pub = self.create_publisher(PointStamped, '/lever_point_arm', 10)

        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        
        self.get_logger().info('System Ready! Publishing to /lever_point_arm even if TF is missing.')

    def depth_callback(self, msg):
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        if self.latest_depth_frame is None: return
        color_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        results = self.model(color_frame, conf=0.4, verbose=False)

        for r in results:
            # صمام الأمان
            if r.keypoints is None or len(r.keypoints.xy) == 0 or len(r.keypoints.xy[0]) < 2:
                continue

            try:
                # 1. استخراج النقاط وحساب الزاوية
                pivot = r.keypoints.xy[0][0]
                tip = r.keypoints.xy[0][1]
                u_p, v_p = int(pivot[0]), int(pivot[1])
                u_t, v_t = int(tip[0]), int(tip[1])

                angle_rad = np.arctan2(v_t - v_p, u_t - u_p)
                angle_deg = np.degrees(angle_rad)

                # 2. حساب العمق (Depth)
                depth_window = self.latest_depth_frame[max(0,v_p-2):v_p+2, max(0,u_p-2):u_p+2]
                valid_depths = depth_window[depth_window > 0]
                
                if len(valid_depths) > 0:
                    depth_z_mm = int(np.median(valid_depths)) - 49 
                    depth_m = float(depth_z_mm) / 1000.0
                    
                    # إحداثيات الكاميرا (Pinhole Math)
                    p_x_cam = depth_m
                    p_y_cam = (self.cx - u_p) * (depth_m / self.fx)
                    p_z_cam = (self.cy - v_p) * (depth_m / self.fy)

                    # 3. إنشاء الرسالة (PointStamped)
                    msg_out = PointStamped()
                    msg_out.header.stamp = self.get_clock().now().to_msg()
                    msg_out.header.frame_id = 'camera_link' # القيمة الافتراضية
                    msg_out.point.x, msg_out.point.y, msg_out.point.z = p_x_cam, p_y_cam, p_z_cam

                    # --- 4. محاولة التحويل (Transform) ---
                    try:
                        transform = self.tf_buffer.lookup_transform('arm_base_link', 'camera_link', rclpy.time.Time())
                        arm_p = do_transform_point(msg_out, transform)
                        
                        # تحديث الرسالة بالقيم الجديدة بعد التحويل
                        msg_out.header.frame_id = 'arm_base_link'
                        msg_out.point = arm_p.point
                        
                        # 5. الفلترة والإرسال للسيريال
                        self.history.append([arm_p.point.x, arm_p.point.y, arm_p.point.z, angle_deg])
                        avg = np.mean(self.history, axis=0)
                        
                        if self.ser and self.ser.is_open:
                            payload = f"<{avg[0]:.3f},{avg[1]:.3f},{avg[2]:.3f},{avg[3]:.1f}>\n"
                            self.ser.write(payload.encode())
                    except:
                        # لو الـ TF فشل، الرسالة هتفضل شايلة إحداثيات الكاميرا (debug mode)
                        pass

                    # --- 6. النشر للـ Topic (خارج بلوك التحويل لضمان الظهور في الـ echo) ---
                    self.point_pub.publish(msg_out)

                    # 7. الرسم
                    cv2.line(color_frame, (u_p, v_p), (u_t, v_t), (255, 0, 0), 2)
                    cv2.circle(color_frame, (u_p, v_p), 5, (0, 0, 255), -1)
                    cv2.putText(color_frame, f"Dist: {depth_z_mm}mm", (u_p + 10, v_p - 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(color_frame, f"Ang: {angle_deg:.1f}deg", (u_p + 10, v_p - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            except Exception as e:
                pass
        
        cv2.imshow("Detection (Diagnosis Mode)", color_frame)
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