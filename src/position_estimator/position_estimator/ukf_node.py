import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UKFNode(Node):
    def __init__(self):
        super().__init__('ukf_node')
        self.subscription_aerial = self.create_subscription(
            String,
            'aerialData',
            self.listener_callback_aerial,
            10)
        self.subscription_celestial = self.create_subscription(
            String,
            'celestialData',
            self.listener_callback_celestial,
            10)
        self.subscription_imu = self.create_subscription(
            String,
            'imuData',
            self.listener_callback_imu,
            10)
        self.subscription_gps = self.create_subscription(
            String,
            'gpsData',
            self.listener_callback_gps,
            10)
        self.publisher = self.create_publisher(String, 'estimated_position', 10)

    def listener_callback_aerial(self, msg):
        self.get_logger().info('Received aerial data: "%s"' % msg.data)

    def listener_callback_celestial(self, msg):
        self.get_logger().info('Received celestial data: "%s"' % msg.data)

    def listener_callback_imu(self, msg):
        self.get_logger().info('Received IMU data: "%s"' % msg.data)

    def listener_callback_gps(self, msg):
        self.get_logger().info('Received GPS data: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = UKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
