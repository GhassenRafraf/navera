import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSNode(Node):
    def __init__(self):
        super().__init__('GPS_node')

        # Define a fixed ID for GPS node
        self.node_id = "gps_node_id"

        # Subscriber
        self.subscription = self.create_subscription(
            NavSatFix,
            'gpsRawData',
            self.listener_callback,
            10)

        # Publisher
        self.publisher = self.create_publisher(NavSatFix, 'gpsData', 10)

    def listener_callback(self, msg):
        # Log received message with the node ID
        self.get_logger().info(f'Received GPS data from node: {self.node_id}')

        # Attach node ID to the frame_id in the header to indicate source
        msg.header.frame_id = self.node_id

        # Publish processed data (still just relaying for now)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
