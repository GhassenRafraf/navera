import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')
        self.publisher_ = self.create_publisher(Imu, 'imuRawData', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        # Template for Imu message publishing
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_frame"

        # Publish the Imu message (without treatment for now)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
