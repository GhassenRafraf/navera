import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
import numpy as np

class ImageAdjustmentNode(Node):
    def __init__(self):
        super().__init__('image_adjustment_node')

        # Initialize variables to store IMU and celestial coordinates data
        self.imu_data = None
        self.celestial_coords = None

        # Subscribers
        self.celestial_coords_sub = self.create_subscription(
            PointStamped,
            'celestialCoords',
            self.celestial_coords_callback,
            10)

        self.imu_raw_data_sub = self.create_subscription(
            Imu,
            'imuRawData',
            self.imu_raw_data_callback,
            10)

        # Publisher
        self.celestial_data_pub = self.create_publisher(
            PointStamped,
            'celestialData',
            10)

    def celestial_coords_callback(self, msg):
        # Store celestial coordinates
        self.celestial_coords = msg
        self.adjust_image()

    def imu_raw_data_callback(self, msg):
        # Store IMU data
        self.imu_data = msg
        self.adjust_image()

    def adjust_image(self):
        # Ensure both IMU and celestial coordinates data are available
        if self.imu_data is None or self.celestial_coords is None:
            return

        # Extract necessary values from IMU message
        ax = self.imu_data.linear_acceleration.x
        ay = self.imu_data.linear_acceleration.y
        az = self.imu_data.linear_acceleration.z

        # Extract celestial coordinates (latitude, longitude, altitude)
        lat = self.celestial_coords.point.x
        lon = self.celestial_coords.point.y
        alt = self.celestial_coords.point.z

        # Apply the image adjustment logic (correct tilt)
        corrected_coords = self.correct_tilt(lat, lon, alt, ax, ay, az)

        # Publish the corrected coordinates
        corrected_msg = PointStamped()
        corrected_msg.header.stamp = self.get_clock().now().to_msg()
        corrected_msg.point.x, corrected_msg.point.y, corrected_msg.point.z = corrected_coords
        self.celestial_data_pub.publish(corrected_msg)

    def correct_tilt(self, lat, lon, alt, ax, ay, az):
        # Placeholder for your image adjustment logic (copied from adjustImagePosition class)
        pitch, roll = self.calculate_pitch_and_roll(ax, ay, az)
        x, y, z = self.gps_to_ecef(lat, lon, alt)
        xyz_coords = np.array([x, y, z])

        R = self.get_rotation_matrix(pitch, roll)

        corrected_coords = np.dot(R, xyz_coords)

        return corrected_coords[0], corrected_coords[1], corrected_coords[2]

    def calculate_pitch_and_roll(self, ax, ay, az):
        # Calculate the pitch and roll angles from accelerometer data
        pitch = np.arctan2(ay, np.sqrt(ax**2 + az**2))
        roll = np.arctan2(-ax, az)
        pitch = np.degrees(pitch)
        roll = np.degrees(roll)
        
        return pitch, roll

    def get_rotation_matrix(self, pitch, roll):
        # Get the rotation matrix from pitch and roll angles
        pitch = np.radians(pitch)
        roll = np.radians(roll)

        R_pitch = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])

        R_roll = np.array([
            [np.cos(roll), 0, np.sin(roll)],
            [0, 1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ])

        R = np.dot(R_pitch, R_roll)
        return R

    def gps_to_ecef(self, lat, lon, alt):
        # Placeholder function for converting GPS to ECEF coordinates
        # Implement your GPS to ECEF conversion logic here
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ImageAdjustmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
