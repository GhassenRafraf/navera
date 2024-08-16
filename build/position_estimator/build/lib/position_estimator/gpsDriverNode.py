import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver_node')
        self.publisher_ = self.create_publisher(NavSatFix, 'gpsRawData', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        # Template for NavSatFix message publishing
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_frame"

        # Publish the NavSatFix message (without treatment for now)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
