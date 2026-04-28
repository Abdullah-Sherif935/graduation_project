import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class AutoImageSaver(Node):
    def __init__(self):
        super().__init__('auto_image_saver')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.counter = 0
        self.save_path = "/home/abdullah/dataset/lever_images"
        os.makedirs(self.save_path, exist_ok=True)
        self.last_save_time = time.time()
        
        # إعدادات التجميع
        self.save_interval = 0.5  # لقطة كل نص ثانية
        self.max_images = 150     # هيقف تلقائياً بعد 150 صورة
        
        print(f"Starting automatic capture in {self.save_path}...")
        print("Move the camera around the door handle smoothly!")

    def listener_callback(self, msg):
        if self.counter >= self.max_images:
            print("Finished capturing dataset! You can close now.")
            rclpy.shutdown()
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera - Auto Capture", frame)
        cv2.waitKey(1)

        current_time = time.time()
        if current_time - self.last_save_time >= self.save_interval:
            filename = f"{self.save_path}/lever_{self.counter}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved: {filename} ({self.counter + 1}/{self.max_images})")
            self.counter += 1
            self.last_save_time = current_time

def main():
    rclpy.init()
    node = AutoImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
