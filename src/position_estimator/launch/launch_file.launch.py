from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='position_estimator',
            executable='gpsDriverNode',
            name='gpsDriverNode',
        ),

        Node(
            package='position_estimator',
            executable='imuDriverNode',
            name='imeDriverNode',
        ),

        Node(
            package='position_estimator',
            executable='celestial_vision_node',
            name='celestial_vision_node',
        ),
        Node(
            package='position_estimator',
            executable='aerial_vision_node',
            name='aerial_vision_node',
        ),
        Node(
            package='position_estimator',
            executable='IMU_node',
            name='IMU_node'
        ),
        Node(
            package='position_estimator',
            executable='GPS_node',
            name='GPS_node'
        ),
        Node(
            package='position_estimator',
            executable='ukf_node',
            name='ukf_node'
        ),
    ])
