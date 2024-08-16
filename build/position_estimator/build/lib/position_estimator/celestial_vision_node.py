import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class CelestialVisionNode(Node):
    def __init__(self):
        super().__init__('celestial_vision_node')
        self.node_id = "celestial_vision_id"
        self.subscription = self.create_subscription(
            Image,
            'celestialRawData',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, 'celestialCoords', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Received image data')
        
        # Template for PoseStamped message publishing
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "celestial_frame"
        
        # Publish the PoseStamped message (without treatment for now)
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CelestialVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
