import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class IMUNode(Node):
    def __init__(self):
        super().__init__('IMU_node')
        self.subscription = self.create_subscription(
            String,
            'imuRawData',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'imuData', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        # Publish processed data (empty for now)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
