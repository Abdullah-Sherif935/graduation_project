import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):

    def __init__(self):

        super().__init__('image_saver')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)

        self.counter = 0

        self.save_path = "/home/abdullah/dataset/images"

        os.makedirs(self.save_path, exist_ok=True)

        print("Press SPACE to capture image")

    def listener_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        cv2.imshow("Camera", frame)

        key = cv2.waitKey(1)

        if key == 32:   # space key

            filename = f"{self.save_path}/img_{self.counter}.jpg"

            cv2.imwrite(filename, frame)

            print("Saved:", filename)

            self.counter += 1


def main():

    rclpy.init()

    node = ImageSaver()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
