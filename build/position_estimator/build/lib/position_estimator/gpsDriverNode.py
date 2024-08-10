import rclpy
from rclpy.node import Node
from std_msgs.msg import String  

class GPSDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver_node')
        self.publisher_ = self.create_publisher(String, 'gpsRawData', 10)
        self.timer = self.create_timer(1.0, self.publish_data)  

    def publish_data(self):
        msg = String()  
        msg.data = self.read_gps_sensor()  
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing GPS data: {msg.data}')

    def read_gps_sensor(self):
        # Replace with actual GPS sensor reading logic
        return "GPS data"

def main(args=None):
    rclpy.init(args=args)
    node = GPSDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
