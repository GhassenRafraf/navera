import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from scipy.linalg import cholesky

class DronePositionEstimatorUKF(Node):
    def __init__(self):
        super().__init__('drone_position_estimator_ukf')
        self.declare_parameter('threshold_percentile', 95)
        self.threshold_percentile = self.get_parameter('threshold_percentile').value

        self.last_known_position = None
        self.kalman_initialized = False

        # Process noise covariance
        self.Q = np.eye(6) * 0.5994

        # Measurement noise covariance
        self.R = np.eye(2) * 4.6415

        # Initial state estimate and covariance
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 5.0

        # Scaling parameter for sigma points
        self.alpha = 1e-3

        # Number of state variables
        self.n = self.x.shape[0]

        # Number of sigma points
        self.num_sigmas = 2 * self.n + 1

        # Weights for sigma points
        self.weights = np.zeros(self.num_sigmas)
        self.weights[0] = self.alpha**2 / (self.n + self.alpha**2)
        self.weights[1:] = 1 / (2 * (self.n + self.alpha**2))

        # Subscribers for data topics
        self.gps_sub = Subscriber(self, Float64MultiArray, 'gpsData')
        self.aerial_sub = Subscriber(self, Float64MultiArray, 'aerialData')
        self.celestial_sub = Subscriber(self, Float64MultiArray, 'celestialData')
        self.imu_sub = Subscriber(self, Float64MultiArray, 'imuData')

        # Synchronizer
        self.ts = ApproximateTimeSynchronizer(
            [self.gps_sub, self.aerial_sub, self.celestial_sub, self.imu_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sensor_callback)

        # Publisher for estimated position
        self.publisher_ = self.create_publisher(Float64MultiArray, 'estimated_position', 10)

    def unscented_transform(self, x, P, kappa=0):
        sigma_points = np.zeros((self.n, self.num_sigmas))
        sigma_points[:, 0] = x.flatten()
        L = cholesky((self.n + kappa) * P).T
        for i in range(self.n):
            sigma_points[:, i + 1] = x.flatten() + L[:, i]
            sigma_points[:, i + 1 + self.n] = x.flatten() - L[:, i]
        return sigma_points

    def acceleration_x_function(self, dt):
        return 0.1 * dt

    def acceleration_y_function(self, dt):
        return 0.2 * dt

    def predict_sigma_points(self, dt=1):
        sigma_points = self.unscented_transform(self.x, self.P, self.alpha)
        for i in range(self.num_sigmas):
            sigma_points[0, i] += sigma_points[2, i] * dt + 0.5 * self.acceleration_x_function(dt) * dt**2
            sigma_points[1, i] += sigma_points[3, i] * dt + 0.5 * self.acceleration_y_function(dt) * dt**2
            sigma_points[2, i] += self.acceleration_x_function(dt)
            sigma_points[3, i] += self.acceleration_y_function(dt)
        return sigma_points

    def predict_mean_and_covariance(self, sigma_points):
        x_pred = np.dot(sigma_points, self.weights).reshape(-1, 1)
        P_pred = np.zeros((self.n, self.n))
        for i in range(self.num_sigmas):
            y = sigma_points[:, i:i + 1] - x_pred
            P_pred += self.weights[i] * np.dot(y, y.T)
        P_pred += self.Q
        return x_pred, P_pred

    def predict(self, dt=1):
        sigma_points = self.predict_sigma_points(dt)
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

        # Publish the estimated position
        msg = Float64MultiArray()
        msg.data = self.x[:2].flatten().tolist()
        self.publisher_.publish(msg)
        return self.x[:2]

    def sensor_callback(self, gps_msg, aerial_msg, celestial_msg, imu_msg):
        # Extract data from messages and treat them all as positional coordinates
        gps_coords = np.array(gps_msg.data).reshape(-1, 2)
        aerial_coords = np.array(aerial_msg.data).reshape(-1, 2)
        celestial_coords = np.array(celestial_msg.data).reshape(-1, 2)
        imu_coords = np.array(imu_msg.data).reshape(-1, 2)  # Assuming IMU provides positional coordinates

        # Combine all sources of data
        combined_coords = np.concatenate((gps_coords, aerial_coords, celestial_coords, imu_coords), axis=0)

        # Estimate position based on combined sensor data
        estimated_position = self.estimate_position(combined_coords)

        # Log estimated position
        self.get_logger().info(f'Estimated Position: {estimated_position.flatten()}')

def main(args=None):
    rclpy.init(args=args)
    node = DronePositionEstimatorUKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
