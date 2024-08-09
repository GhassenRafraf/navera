import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import math

class GpsToEcefConverter(Node):
    def __init__(self):
        super().__init__('gps_to_ecef_converter')
        self.subscription = self.create_subscription(
            NavSatFix,
            'raw_gps_data',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'gpsData',
            10
        )
        self.get_logger().info('GPS to ECEF Converter node has been started.')

    def listener_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        
        x, y, z = self.gps_to_ecef(lat, lon, alt)
        
        msg_out = Float64MultiArray()
        msg_out.data = [x, y, z]
        
        self.publisher.publish(msg_out)
        self.get_logger().info(f'Published ECEF coordinates: x={x}, y={y}, z={z}')

    @staticmethod
    def gps_to_ecef(lat, lon, alt):
        # WGS84 ellipsoid constants
        a = 6378137.0  # Semi-major axis
        f = 1 / 298.257223563  # Flattening
        e2 = f * (2 - f)  # Square of eccentricity

        lat = math.radians(lat)
        lon = math.radians(lon)

        N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)

        x = (N + alt) * math.cos(lat) * math.cos(lon)
        y = (N + alt) * math.cos(lat) * math.sin(lon)
        z = (N * (1 - e2) + alt) * math.sin(lat)

        return x, y, z

def main(args=None):
    rclpy.init(args=args)
    gps_to_ecef_converter = GpsToEcefConverter()
    rclpy.spin(gps_to_ecef_converter)
    gps_to_ecef_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
