# Navera

This project is a drone position estimation, integrating data from multiple subsystems such as GPS, IMU, aerial, and celestial computer vision systems.

## Project Overview

The project is designed to estimate the position of a drone by processing GPS coordinates from multiple sources, including IMU data, aerial vision, and celestial vision. The project runs on ROS2 and is built for deployment on a Jetson Orin NX using Docker. The system handles non-linear dynamics and measurements with outlier detection for robust performance in real-world environments.

## Prerequisites

- ROS2 Humble (or your preferred distribution)
- Python 3.x

## Clone the Repository

To clone the project repository, run the following command:

```bash
git clone https://github.com/GhassenRafraf/navera.git
cd navera
```

## Build the Project

```bash
cd ~/navera
colcon build
```

## Source the Workspace
After building the workspace, source the setup script to ensure ROS2 can locate the package:

```bash
source ~/navera/install/setup.bash
```

## Launching the entire system with the specified launch file

```bash
ros2 launch position_estimator launch_file.py
```
