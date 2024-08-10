import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GPSNode(Node):
    def __init__(self):
        super().__init__('GPS_node')
        self.subscription = self.create_subscription(
            String,
            'gpsRawData',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'gpsData', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        # Publish processed data (empty for now)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
