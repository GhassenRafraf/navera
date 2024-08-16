import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
import math

class EcefToGpsNode(Node):
    def __init__(self):
        super().__init__('ecef_to_gps_node')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'ecef_coordinates',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            NavSatFix,
            'gps_coordinates',
            10
        )
        self.get_logger().info('ECEF to GPS Node has been started.')

    def listener_callback(self, msg):
        x, y, z = msg.data
        
        lat, lon, alt = EcefToGpsNode.ecef_to_gps(x, y, z)
        
        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        gps_msg.status.status = gps_msg.status.STATUS_FIX
        gps_msg.position_covariance_type = gps_msg.position_covariance_type.COVARIANCE_TYPE_UNKNOWN

        self.publisher.publish(gps_msg)
        self.get_logger().info(f'Published GPS coordinates: lat={lat}, lon={lon}, alt={alt}')

def main(args=None):
    rclpy.init(args=args)
    ecef_to_gps_node = EcefToGpsNode()
    rclpy.spin(ecef_to_gps_node)
    ecef_to_gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
