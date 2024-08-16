import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.linalg import cholesky

class DronePositionEstimatorUKF:
    def __init__(self, threshold_percentile=95):
        self.threshold_percentile = threshold_percentile
        self.last_known_position = None
        self.kalman_initialized = False

        self.Q = np.eye(6) * 0.5994  
        self.R = np.eye(2) * 4.6415  

        self.x = np.zeros((6, 1))  
        self.P = np.eye(6) * 5.0  

        self.alpha = 1e-3
        self.n = self.x.shape[0]
        self.num_sigmas = 2 * self.n + 1

        self.weights = np.zeros(self.num_sigmas)
        self.weights[0] = self.alpha**2 / (self.n + self.alpha**2)
        self.weights[1:] = 1 / (2 * (self.n + self.alpha**2))

    def unscented_transform(self, x, P, kappa=0):
        sigma_points = np.zeros((self.n, self.num_sigmas))
        sigma_points[:, 0] = x.flatten()
        L = cholesky((self.n + kappa) * P).T
        for i in range(self.n):
            sigma_points[:, i + 1] = x.flatten() + L[:, i]
            sigma_points[:, i + 1 + self.n] = x.flatten() - L[:, i]
        return sigma_points

    def predict_sigma_points(self, dt=1, accel_x=0, accel_y=0):
        sigma_points = self.unscented_transform(self.x, self.P, self.alpha)
        for i in range(self.num_sigmas):
            sigma_points[0, i] += sigma_points[2, i] * dt + 0.5 * accel_x * dt**2
            sigma_points[1, i] += sigma_points[3, i] * dt + 0.5 * accel_y * dt**2
            sigma_points[2, i] += accel_x * dt  
            sigma_points[3, i] += accel_y * dt  
        return sigma_points

    def predict_mean_and_covariance(self, sigma_points):
        x_pred = np.dot(sigma_points, self.weights).reshape(-1, 1)
        P_pred = np.zeros((self.n, self.n))
        for i in range(self.num_sigmas):
            y = sigma_points[:, i:i + 1] - x_pred
            P_pred += self.weights[i] * np.dot(y, y.T)
        P_pred += self.Q
        return x_pred, P_pred

    def predict(self, dt=1, accel_x=0, accel_y=0):
        sigma_points = self.predict_sigma_points(dt, accel_x, accel_y)
        x_pred, P_pred = self.predict_mean_and_covariance(sigma_points)
        return x_pred, P_pred

    def update(self, z):
        sigma_points = self.predict_sigma_points()
        x_pred, P_pred = self.predict_mean_and_covariance(sigma_points)

        z_pred = np.dot(sigma_points[:2, :], self.weights).reshape(-1, 1)
        Pzz = np.zeros((2, 2))
        for i in range(self.num_sigmas):
            z_diff = sigma_points[:2, i:i + 1] - z_pred
            Pzz += self.weights[i] * np.dot(z_diff, z_diff.T)
        Pzz += self.R

        Pxz = np.zeros((self.n, 2))
        for i in range(self.num_sigmas):
            x_diff = sigma_points[:, i:i + 1] - x_pred
            z_diff = sigma_points[:2, i:i + 1] - z_pred
            Pxz += self.weights[i] * np.dot(x_diff, z_diff.T)

        K = np.dot(Pxz, np.linalg.inv(Pzz))
        self.x = x_pred + np.dot(K, (z - z_pred))
        self.P = P_pred - np.dot(K, np.dot(Pzz, K.T))

    def mahalanobis_distance(self, point, mean, cov_inv):
        diff = point - mean
        return np.sqrt(diff.T @ cov_inv @ diff)

    def detect_outliers(self, coords):
        if len(coords) < 3:
            return np.zeros(len(coords)), 0
        
        mean = np.mean(coords, axis=0)
        cov = np.cov(coords, rowvar=False)
        cov_inv = np.linalg.inv(cov)
        
        distances = np.array([self.mahalanobis_distance(coord, mean, cov_inv) for coord in coords])
        threshold = np.percentile(distances, self.threshold_percentile)
        
        return distances, threshold

    def estimate_position(self, coords, dt=1):
        distances, threshold = self.detect_outliers(coords)
        
        filtered_coords = []
        weights = []
        for i, coord in enumerate(coords):
            if distances[i] <= threshold:
                weight = 1 / distances[i] if distances[i] != 0 else 1
                filtered_coords.append(coord)
                weights.append(weight)
        
        weights = np.array(weights, dtype=float)
        weights /= np.sum(weights)
        
        filtered_coords = np.array(filtered_coords)
        weighted_position = np.dot(filtered_coords.T, weights).flatten()

        if not self.kalman_initialized:
            self.x[:2] = weighted_position.reshape(2, 1)
            self.kalman_initialized = True
        else:
            self.update(weighted_position.reshape(2, 1))

        self.last_known_position = self.x[:2]
        return self.x[:2]

class UKFNode(Node):
    def __init__(self):
        super().__init__('ukf_node')

        # Define a fixed ID for this node
        self.node_id = "ukf_node_id"

        # Subscribers
        self.subscription_imu = self.create_subscription(
            Imu,
            'imuRawData',
            self.imu_callback,
            10
        )
        self.subscription_celestial = self.create_subscription(
            PointStamped,
            'celestialData',
            self.point_callback,
            10
        )
        self.subscription_aerial = self.create_subscription(
            PointStamped,
            'aerialData',
            self.point_callback,
            10
        )
        self.subscription_gps = self.create_subscription(
            PointStamped,
            'gpsData',
            self.point_callback,
            10
        )
        self.subscription_gps = self.create_subscription(
            PointStamped,
            'imuData',
            self.point_callback,
            10
        )

        # Publisher
        self.publisher = self.create_publisher(
            PointStamped,
            'estimated_position',
            10
        )

        # UKF estimator
        self.ukf_estimator = DronePositionEstimatorUKF()
        self.received_data = []
        self.last_imu_data = None
        self.get_logger().info('UKF Node has been started.')

    def imu_callback(self, msg):
        # Extract acceleration from IMU data
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y

        self.last_imu_data = (accel_x, accel_y)

    def point_callback(self, msg):
        # Collect data from each topic
        self.received_data.append(np.array([msg.point.x, msg.point.y]).reshape(1, -1))

        # Check if data from all topics have been received
        if len(self.received_data) == 5 and self.last_imu_data:
            # Concatenate all received data
            all_coords = np.concatenate(self.received_data, axis=0)
            accel_x, accel_y = self.last_imu_data

            # Estimate position using UKF
            estimated_position = self.ukf_estimator.predict(all_coords, accel_x=accel_x, accel_y=accel_y)

            # Prepare the message to be published
            estimated_position_msg = PointStamped()
            estimated_position_msg.header.stamp = self.get_clock().now().to_msg()
            estimated_position_msg.header.frame_id = self.node_id
            estimated_position_msg.point.x, estimated_position_msg.point.y = estimated_position.flatten()

            # Publish the estimated position
            self.publisher.publish(estimated_position_msg)
            self.get_logger().info(f'Published estimated position: {estimated_position.flatten().tolist()}')

            # Clear the data for the next round
            self.received_data = []

def main(args=None):
    rclpy.init(args=args)
    ukf_node = UKFNode()
    rclpy.spin(ukf_node)
    ukf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
