import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointExtractor(Node):

    def __init__(self):
        super().__init__('point_extractor')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.listener_callback,
            10)

    def listener_callback(self, msg):

        points = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)

        for i, p in enumerate(points):

            if i == 100000:   # نقطة من السحابة

                x,y,z = p

                print("------------")
                print("3D POINT")
                print("X:",x)
                print("Y:",y)
                print("Z:",z)
                print("------------")

                break

def main():

    rclpy.init()

    node = PointExtractor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
