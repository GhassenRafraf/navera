import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped

class IMUNode(Node):
    def __init__(self):
        super().__init__('IMU_node')

        # Define a fixed ID for this node
        self.node_id = "imu_node_id"

        # Subscriber
        self.subscription = self.create_subscription(
            Imu,
            'imuRawData',
            self.listener_callback,
            10)

        # Publisher
        self.publisher = self.create_publisher(PointStamped, 'imuData', 10)

    def listener_callback(self, msg):
        # Log received IMU data and assign node ID to the header
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.node_id

        # Placeholder for processing IMU data and populating PointStamped
        self.publisher.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
